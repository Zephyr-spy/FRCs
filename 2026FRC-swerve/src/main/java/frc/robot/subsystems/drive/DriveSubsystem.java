package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.pathplanner.auto.AutoBuilder;
import frc.lib.pathplanner.commands.PathPlannerAuto;
import frc.lib.pathplanner.config.ModuleConfig;
import frc.lib.pathplanner.config.PIDConstants;
import frc.lib.pathplanner.config.RobotConfig;
import frc.lib.pathplanner.controllers.PPHolonomicDriveController;
import frc.lib.pathplanner.path.PathConstraints;
import frc.lib.pathplanner.pathfinding.Pathfinding;
import frc.lib.pathplanner.trajectory.PathPlannerTrajectory;
import frc.lib.pathplanner.util.FlippingUtil;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ModuleConstants;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.config.RobotSettings;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import org.littletonrobotics.junction.Logger;

/** Shared 20 ms estimator/control boundary for REV hardware, physics and log replay. */
public class DriveSubsystem extends SubsystemBase {
  private final DriveIO io;
  private final GyroIO gyro;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final RobotState state;
  private final SwerveDrivePoseEstimator estimator;
  private final Field2d field = new Field2d();
  private Rotation2d odometryHeading = new Rotation2d();
  private boolean wasTrusted;
  private boolean hadUsableModules;
  private double lastTelemetry = -1;
  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
  private String commandStatus = "停止";

  private final PPHolonomicDriveController pathController = new PPHolonomicDriveController(
      new PIDConstants(AutoConstants.kPXController,0,0),new PIDConstants(AutoConstants.kPYController,0,0),
      new PIDConstants(AutoConstants.kPThetaController,0,0));
  private PathPlannerTrajectory pathTrajectory;
  private double pathStartTime;
  private boolean pathPlannerConfigured;

  public DriveSubsystem(DriveIO io, GyroIO gyro, RobotState state) {
    this.io = io; this.gyro = gyro; this.state = state;
    estimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
        odometryHeading, positions(), new Pose2d());
    SmartDashboard.putData("Field", field);
    configurePathPlanner();
  }
  public static Translation2d[] moduleLocations() {
    double x = DriveConstants.kWheelBase / 2, y = DriveConstants.kTrackWidth / 2;
    return new Translation2d[] {new Translation2d(x,y), new Translation2d(x,-y),
        new Translation2d(-x,y), new Translation2d(-x,-y)};
  }
  private SwerveModulePosition[] positions() {
    SwerveModulePosition[] result = new SwerveModulePosition[4];
    for (int i=0;i<4;i++) result[i] = new SwerveModulePosition(inputs.positionsMeters[i],
        Rotation2d.fromRadians(inputs.anglesRad[i]));
    return result;
  }
  private SwerveModuleState[] states() {
    SwerveModuleState[] result = new SwerveModuleState[4];
    for (int i=0;i<4;i++) result[i] = new SwerveModuleState(inputs.velocitiesMps[i],
        Rotation2d.fromRadians(inputs.anglesRad[i]));
    return result;
  }
  @Override public void periodic() {
    io.readInputs(inputs); gyro.readInputs(gyroInputs);
    Logger.processInputs("DriveInputs", inputs);
    Logger.processInputs("GyroInputs", gyroInputs);
    double now = Timer.getTimestamp();
    boolean trusted = hasHeading();
    if (trusted) odometryHeading = Rotation2d.fromRadians(
        gyroInputs.yawRad * (DriveConstants.kGyroReversed ? -1 : 1));
    if (inputs.usable) {
      // Sensor recovery establishes a new baseline, never applies a discontinuous gyro jump.
      if (trusted != wasTrusted || !hadUsableModules) {
        estimator.resetPosition(odometryHeading, positions(), getPose());
        state.clearHistory();
      }
      estimator.updateWithTime(now, odometryHeading, positions());
    }
    wasTrusted = trusted; hadUsableModules = inputs.usable;
    state.setHeadingTrusted(trusted && inputs.usable);
    state.addOdometryMeasurement(now, getPose());
    ChassisSpeeds measured = inputs.usable ? DriveConstants.kDriveKinematics.toChassisSpeeds(states()) : new ChassisSpeeds();
    double yawRate = trusted ? gyroInputs.yawRateRadPerSec * (DriveConstants.kGyroReversed ? -1 : 1) : Double.NaN;
    var measuredField = ChassisSpeeds.fromRobotRelativeSpeeds(measured, getPose().getRotation());
    state.addDriveMotionMeasurements(now,Double.NaN,Double.NaN,yawRate,
        Double.NaN,Double.NaN,Double.NaN,Double.NaN,desiredSpeeds,
        ChassisSpeeds.fromRobotRelativeSpeeds(desiredSpeeds,getPose().getRotation()),
        measured,measuredField,measuredField);
    state.incrementIterationCount();
    if (!DriverStation.isEnabled() || !inputs.usable) stop();
    // Follow paths after this cycle's sensor/pose update, on the same 20 ms loop.
    updatePathPlanner();
    field.setRobotPose(getPose());
    Logger.recordOutput("Drive/Pose", getPose());
    Logger.recordOutput("Drive/ModuleStates", states());
    Logger.recordOutput("Drive/Status", inputs.status);
    if (now-lastTelemetry >= .2) { lastTelemetry=now; publishTelemetry(now); }
  }
  // PathPlanner initialization is called once at the end of this subsystem's constructor.
  // This project uses the template's local fork; following runs here, not in another subsystem.
  private void configurePathPlanner() {
    FlippingUtil.fieldSizeX=Constants.kAprilTagLayout.getFieldLength();
    FlippingUtil.fieldSizeY=Constants.kAprilTagLayout.getFieldWidth();
    Pathfinding.setAllowed(DriveSubsystem::navgridReady);
    if(RobotSettings.Paths.enabled && RobotSettings.modelReady()) {
      var module=new ModuleConfig(ModuleConstants.kWheelDiameterMeters/2,
          Constants.DriveConstants.kMaxSpeedMetersPerSecond,RobotSettings.Model.wheelCOF,
          DCMotor.getNEO(1).withReduction(ModuleConstants.kDrivingMotorReduction),50,1);
      var robot=new RobotConfig(RobotSettings.Model.massKg,RobotSettings.Model.moiKgMetersSquared,
          module,RobotSettings.Model.cogHeightMeters,DriveSubsystem.moduleLocations());
      AutoBuilder.configure(this::getPose,this::resetOdometry,this::getMeasuredSpeeds,
          this::acceptTrajectory,robot,state::isRedAlliance,this);
      pathPlannerConfigured=true;
    }
    SmartDashboard.putString("PathPlanner/Status",pathPlannerConfigured?"模型就緒；Auto 預設停止":"停用：請填物理模型並啟用 Paths.enabled");
    SmartDashboard.putBoolean("PathPlanner/Pathfinding Ready",navgridReady());
  }
  private void acceptTrajectory(PathPlannerTrajectory next) {
    if(next==null || next.isStayStoppedTrajectory() || next.getStates().isEmpty()
        || !Double.isFinite(next.getTotalTimeSeconds())) {stop();return;}
    pathTrajectory=next;pathStartTime=Timer.getTimestamp();
    pathController.reset(getPose(),getMeasuredSpeeds());
  }
  private void updatePathPlanner() {
    if(pathTrajectory==null)return;
    if(!DriverStation.isAutonomousEnabled() || !readyForAuto()) {stop();return;}
    var target=pathTrajectory.sample(Math.min(Timer.getTimestamp()-pathStartTime,pathTrajectory.getTotalTimeSeconds()));
    var output=pathController.calculateRobotRelativeSpeeds(getPose(),target);
    double norm=Math.hypot(output.vxMetersPerSecond,output.vyMetersPerSecond);
    if(norm>AutoConstants.kMaxSpeedMetersPerSecond) {
      output.vxMetersPerSecond*=AutoConstants.kMaxSpeedMetersPerSecond/norm;
      output.vyMetersPerSecond*=AutoConstants.kMaxSpeedMetersPerSecond/norm;
    }
    output.omegaRadiansPerSecond=MathUtil.clamp(output.omegaRadiansPerSecond,
        -AutoConstants.kMaxAngularSpeedRadiansPerSecond,AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    // REV's existing kV remains the sole motor feedforward; CTRE wheel-force feedforwards are not used.
    setRobotRelativeSpeeds(output);
  }
  public List<String> getPathPlannerAutoNames() {
    if(!pathPlannerConfigured)return List.of();
    Path directory=Filesystem.getDeployDirectory().toPath().resolve("pathplanner/autos");
    if(!Files.isDirectory(directory))return List.of();
    try(var files=Files.list(directory)) {
      return files.map(p->p.getFileName().toString()).filter(n->n.endsWith(".auto"))
          .map(n->n.substring(0,n.length()-5)).sorted().toList();
    }catch(java.io.IOException e){return List.of();}
  }
  public Command buildPathPlannerAuto(String name) {
    if(!pathPlannerConfigured)return rejected("PathPlanner 模型尚未設定");
    try {
      // A file must respect this robot's existing limits, including every constraint zone.
      for(var path:PathPlannerAuto.getPathGroupFromAutoFile(name)) {
        validateConstraints(path.getGlobalConstraints());
        for(var zone:path.getConstraintZones())validateConstraints(zone.constraints());
        if(path.getGoalEndState().velocityMPS()>AutoConstants.kMaxSpeedMetersPerSecond)
          throw new IllegalArgumentException("路徑終點速度超過現有 Auto 上限");
      }
      return AutoBuilder.buildAuto(name).until(()->!readyForAuto() || !DriverStation.isAutonomousEnabled())
          .finallyDo(interrupted->stop());
    }catch(Exception e){return rejected("路徑無法使用："+e.getMessage());}
  }
  public Command pathfindTo(Pose2d target) {
    if(!pathPlannerConfigured || !navgridReady())return rejected("尋路停用：2026 障礙地圖未確認");
    return AutoBuilder.pathfindToPose(target,new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared,AutoConstants.kMaxAngularSpeedRadiansPerSecond,
        AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared,AutoConstants.kMaxAccelerationMetersPerSecondSquared),0)
        .until(()->!readyForAuto() || !DriverStation.isAutonomousEnabled()).finallyDo(interrupted->stop());
  }
  private Command rejected(String message){return Commands.runOnce(()->{stop();report(message);},this);}
  public static void validateConstraints(PathConstraints c) {
    if(!Double.isFinite(c.maxXAccelMPSSq()) || !Double.isFinite(c.maxYAccelMPSSq())
        || c.maxXAccelMPSSq()<=0 || c.maxYAccelMPSSq()<=0
        || c.maxXAccelMPSSq()>AutoConstants.kMaxAccelerationMetersPerSecondSquared
        || c.maxYAccelMPSSq()>AutoConstants.kMaxAccelerationMetersPerSecondSquared || c.unlimited() || !Double.isFinite(c.maxVelocityMPS()) || !Double.isFinite(c.maxAccelerationMPSSq())
        || !Double.isFinite(c.maxAngularVelocityRadPerSec()) || !Double.isFinite(c.maxAngularAccelerationRadPerSecSq())
        || c.maxVelocityMPS()<=0 || c.maxAccelerationMPSSq()<=0 || c.maxAngularVelocityRadPerSec()<=0
        || c.maxAngularAccelerationRadPerSecSq()<=0
        || c.maxVelocityMPS()>AutoConstants.kMaxSpeedMetersPerSecond
        || c.maxAccelerationMPSSq()>AutoConstants.kMaxAccelerationMetersPerSecondSquared
        || c.maxAngularVelocityRadPerSec()>AutoConstants.kMaxAngularSpeedRadiansPerSecond
        || c.maxAngularAccelerationRadPerSecSq()>AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared)
      throw new IllegalArgumentException("路徑約束超過 Constants.AutoConstants，請在 PathPlanner 調整");
  }
  public static boolean navgridReady() {
    if(!RobotSettings.Paths.navgridConfirmed || !RobotSettings.modelReady())return false;
    Path path=Filesystem.getDeployDirectory().toPath().resolve("pathplanner/navgrid.json");
    try(var reader=Files.newBufferedReader(path)) {
      var json=(org.json.simple.JSONObject)new org.json.simple.parser.JSONParser().parse(reader);
      var size=(org.json.simple.JSONObject)json.get("field_size");
      double x=((Number)size.get("x")).doubleValue(),y=((Number)size.get("y")).doubleValue();
      double node=((Number)json.get("nodeSizeMeters")).doubleValue();
      var rows=(org.json.simple.JSONArray)json.get("grid");
      if(!Double.isFinite(x)||!Double.isFinite(y)||!RobotSettings.positive(node)
          || Math.abs(x-Constants.kAprilTagLayout.getFieldLength())>.05
          || Math.abs(y-Constants.kAprilTagLayout.getFieldWidth())>.05
          || rows.size()!=(int)Math.ceil(y/node))return false;
      boolean obstacle=false;
      for(Object row:rows) {
        if(!(row instanceof org.json.simple.JSONArray cols)||cols.size()!=(int)Math.ceil(x/node))return false;
        for(Object value:cols){if(!(value instanceof Boolean))return false;obstacle|=(boolean)value;}
      }
      return obstacle;
    }catch(Exception e){return false;}
  }
  public boolean hasHeading() { return gyroInputs.trusted && gyroInputs.connected; }
  public boolean readyForAuto() { return hasHeading() && inputs.usable; }
  public Pose2d getPose() { return estimator.getEstimatedPosition(); }
  public ChassisSpeeds getMeasuredSpeeds() { return DriveConstants.kDriveKinematics.toChassisSpeeds(states()); }
  public void resetOdometry(Pose2d pose) {
    estimator.resetPosition(odometryHeading,positions(),pose);
    state.clearHistory();
    state.addOdometryMeasurement(Timer.getTimestamp(),pose);
  }
  public void zeroHeading() {
    if (!hasHeading()) { report("無可靠 Pigeon2 航向，無法歸零"); stop(); return; }
    stop(); resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }
  /** SI units; a missing gyro deliberately switches only manual driving to robot relative. */
  public void driveManual(double forward, double strafe, double rotation) {
    var speeds = hasHeading()
        ? ChassisSpeeds.fromFieldRelativeSpeeds(forward,strafe,rotation,getPose().getRotation())
        : new ChassisSpeeds(forward,strafe,rotation);
    setRobotRelativeSpeeds(speeds);
    report(hasHeading() ? "場地導向" : "車身導向：" + gyroInputs.status);
  }
  public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
    if (!finite(speeds) || !inputs.usable || !DriverStation.isEnabled()) { stop(); return; }
    double max = DriveConstants.kMaxSpeedMetersPerSecond;
    double scale = Math.min(1, max / Math.max(max, Math.hypot(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond)));
    desiredSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond*scale, speeds.vyMetersPerSecond*scale,
        MathUtil.clamp(speeds.omegaRadiansPerSecond,-DriveConstants.kMaxAngularSpeed,DriveConstants.kMaxAngularSpeed));
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(desiredSpeeds));
  }
  public void setModuleStates(SwerveModuleState[] states) {
    if (!DriverStation.isEnabled() || !inputs.usable) { stop(); return; }
    if (states.length != 4) throw new IllegalArgumentException("需要四個輪組");
    for (var s:states) if (!Double.isFinite(s.speedMetersPerSecond) || !Double.isFinite(s.angle.getRadians())) {stop();return;}
    SwerveDriveKinematics.desaturateWheelSpeeds(states,DriveConstants.kMaxSpeedMetersPerSecond);
    io.setModuleStates(states);
  }
  public void setX() {
    desiredSpeeds = new ChassisSpeeds();
    setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(0,Rotation2d.fromDegrees(45)),new SwerveModuleState(0,Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0,Rotation2d.fromDegrees(-45)),new SwerveModuleState(0,Rotation2d.fromDegrees(45))});
  }
  public void stop() { pathTrajectory=null; desiredSpeeds=new ChassisSpeeds(); io.stop(); }
  public void report(String message) { commandStatus=message; SmartDashboard.putString("Swerve/Control Status",message); }
  public void addVisionMeasurement(VisionFieldPoseEstimate estimate) {
    if (!readyForAuto()) return;
    estimator.addVisionMeasurement(estimate.getVisionRobotPoseMeters(),estimate.getTimestampSeconds(),
        estimate.getVisionMeasurementStdDevs());
  }
  private static boolean finite(ChassisSpeeds s) {
    return Double.isFinite(s.vxMetersPerSecond)&&Double.isFinite(s.vyMetersPerSecond)&&Double.isFinite(s.omegaRadiansPerSecond);
  }
  private void publishTelemetry(double now) {
    String[] names={"FrontLeft","FrontRight","RearLeft","RearRight"};
    int[] ids={DriveConstants.kFrontLeftDrivingCanId,DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontRightDrivingCanId,DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kRearLeftDrivingCanId,DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kRearRightDrivingCanId,DriveConstants.kRearRightTurningCanId};
    for(int i=0;i<4;i++) {
      String root="Swerve/"+names[i];
      for(int j=0;j<2;j++) {
        int n=2*i+j; String p=root+(j==0?"/Drive/":"/Turn/");
        SmartDashboard.putNumber(p+"CAN ID",ids[n]);
        SmartDashboard.putNumber(p+"Bus Voltage V",inputs.voltage[n]);
        SmartDashboard.putNumber(p+"Applied Output",inputs.output[n]);
        SmartDashboard.putNumber(p+"Current A",inputs.current[n]);
        SmartDashboard.putNumber(p+"Temperature C",inputs.temperature[n]);
        SmartDashboard.putNumber(p+"Fault Bits",inputs.faults[n]);
        SmartDashboard.putNumber(p+"Warning Bits",inputs.warnings[n]);
        SmartDashboard.putBoolean(p+"Has Fault",inputs.faults[n]!=0);
        SmartDashboard.putBoolean(p+"Sensor Fault",(inputs.faults[n]&4)!=0);
        SmartDashboard.putBoolean(p+"CAN Fault",(inputs.faults[n]&8)!=0);
        SmartDashboard.putBoolean(p+"Sensor Warning",(inputs.warnings[n]&16)!=0);
        SmartDashboard.putBoolean(p+"Stall Warning",(inputs.warnings[n]&32)!=0);
        SmartDashboard.putString(p+"Initialization Status",inputs.initStatus[n]);
        SmartDashboard.putString(p+"Command Status",inputs.commandStatus[n]);
        SmartDashboard.putString(p+"Voltage Read Status",inputs.voltageReadStatus[n]);
      }
      String d=root+"/Drive/", t=root+"/Turn/";
      SmartDashboard.putNumber(d+"Position m",inputs.positionsMeters[i]);
      SmartDashboard.putNumber(d+"Velocity mps",inputs.velocitiesMps[i]);
      SmartDashboard.putNumber(d+"Target mps",inputs.targetSpeedsMps[i]);
      SmartDashboard.putString(d+"Position Read Status",inputs.positionReadStatus[2*i]);
      SmartDashboard.putString(d+"Velocity Read Status",inputs.velocityReadStatus[2*i]);
      SmartDashboard.putString(t+"Encoder Read Status",inputs.readStatus[2*i+1]);
      SmartDashboard.putNumber(t+"Absolute Position deg",Math.toDegrees(inputs.absoluteAnglesRad[i]));
      SmartDashboard.putNumber(t+"Chassis Position deg",Math.toDegrees(MathUtil.angleModulus(inputs.anglesRad[i])));
      SmartDashboard.putNumber(t+"Target Absolute deg",Math.toDegrees(inputs.targetAbsoluteRad[i]));
      SmartDashboard.putNumber(t+"Error deg",inputs.hasTarget[i]?Math.toDegrees(MathUtil.angleModulus(inputs.targetAbsoluteRad[i]-inputs.absoluteAnglesRad[i])):0);
      SmartDashboard.putBoolean(root+"/Has Command Target",inputs.hasTarget[i]);
    }
    SmartDashboard.putBoolean("Swerve/Robot Enabled",DriverStation.isEnabled());
    SmartDashboard.putNumber("Swerve/Telemetry Timestamp s",now);
    SmartDashboard.putString("Swerve/IO Status",inputs.status);
    SmartDashboard.putString("Swerve/Gyro/Status",gyroInputs.status);
    SmartDashboard.putBoolean("Swerve/Gyro/Connected",gyroInputs.connected);
    SmartDashboard.putBoolean("Swerve/Gyro/Trusted",hasHeading());
    SmartDashboard.putString("Swerve/Control Status",commandStatus);
    SmartDashboard.putString("Swerve/Auto Status",readyForAuto()?"感測器就緒；仍需明確選取 Auto":"停用：輪組或 Pigeon 航向不可信");
    SmartDashboard.putString("Swerve/Heading Hold Status",!RobotSettings.Heading.enabled?"關閉（預設）":
        !RobotSettings.complete(RobotSettings.Heading.class) || !RobotSettings.positive(RobotSettings.Heading.p)
        || RobotSettings.Heading.i<0 || RobotSettings.Heading.d<0?"停用：PID 未設定或無效":
        !hasHeading()?"停用：無可信航向":"已啟用（僅手動命令）");

  }
}
