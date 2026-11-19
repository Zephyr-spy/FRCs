package frc.robot;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.config.RobotSettings;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.simulation.*;
import java.util.List;
import java.util.Set;

public final class RobotContainer {
  private final RobotState state = new RobotState(this::acceptVision);
  private final SimulatedDriveState simulatedState = new SimulatedDriveState();
  private final DriveSubsystem drive;
  private final VisionSubsystem vision;
  private final CommandXboxController controller = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final org.littletonrobotics.junction.networktables.LoggedDashboardChooser<String> auto =
      new org.littletonrobotics.junction.networktables.LoggedDashboardChooser<>("Auto Selector");

  public RobotContainer() {
    if (Robot.isReplay()) {
      drive = new DriveSubsystem(new DriveIO(){},new GyroIO(){},state);
    } else if (RobotBase.isReal()) {
      drive = new DriveSubsystem(new DriveIORev(),new GyroIOPigeon2(),state);
    } else if (RobotSettings.physicsReady()) {
      var physics = new DriveIOMapleSim(simulatedState);
      drive = new DriveSubsystem(physics,physics.gyroIO(),state);
    } else {
      drive = new DriveSubsystem(new DriveIO(){},new GyroIO(){},state);
    }
    VisionIO visionIO = inputs -> {};
    if (!Robot.isReplay() && RobotSettings.cameraReady()) {
      if (RobotBase.isReal()) visionIO = new VisionIOHardwareLimelight();
      else if (RobotSettings.cameraSimReady()) visionIO = new VisionIOSimPhoton(simulatedState);
    }
    vision = new VisionSubsystem(visionIO,state);
    vision.setUseVision(Robot.isReplay() || RobotSettings.cameraReady());
    drive.setDefaultCommand(new ManualDriveCommand(drive,this::manualEnabled,
        ()->controller.getLeftY(),()->controller.getLeftX(),()->controller.getRightX()));
    controller.rightBumper().and(this::manualEnabled)
        .whileTrue(Commands.runEnd(drive::setX,drive::stop,drive));
    controller.start().and(this::manualEnabled).and(()->!controller.getHID().getRightBumperButton())
        .onTrue(Commands.runOnce(drive::zeroHeading,drive));
    auto.addDefaultOption("停止（預設）","STOP");
    auto.addOption("S 型測試：需可靠航向","S_CURVE");
    for (String name : drive.getPathPlannerAutoNames()) auto.addOption("PathPlanner: "+name,"PATH:"+name);
    publishConfiguration();
  }
  private boolean manualEnabled() {return DriverStation.isTeleopEnabled() && controller.getHID().isConnected();}
  public Command getAutonomousCommand() {
    return Commands.defer(()-> {
      String selection=auto.get();
      if(selection==null || selection.equals("STOP")) return Commands.run(drive::stop,drive);
      if(!drive.readyForAuto()) return Commands.runOnce(()->{drive.stop();drive.report("Auto 停用：底盤或 Pigeon2 尚未就緒");},drive);
      if(selection.equals("S_CURVE")) return sCurve();
      if(selection.startsWith("PATH:")) return drive.buildPathPlannerAuto(selection.substring(5));
      return Commands.runOnce(drive::stop,drive);
    },Set.of(drive)).finallyDo(interrupted->stop());
  }
  private Command sCurve() {
    var config=new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);
    var trajectory=TrajectoryGenerator.generateTrajectory(new Pose2d(),
        List.of(new Translation2d(1,1),new Translation2d(2,-1)),new Pose2d(3,0,new Rotation2d()),config);
    var theta=new ProfiledPIDController(AutoConstants.kPThetaController,0,0,AutoConstants.kThetaControllerConstraints);
    theta.enableContinuousInput(-Math.PI,Math.PI);
    return Commands.runOnce(()->drive.resetOdometry(trajectory.getInitialPose()),drive)
        .andThen(new SwerveControllerCommand(trajectory,drive::getPose,DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController,0,0),new PIDController(AutoConstants.kPYController,0,0),
            theta,drive::setModuleStates,drive))
        .until(()->!drive.readyForAuto() || !DriverStation.isAutonomousEnabled())
        .finallyDo(interrupted->drive.stop());
  }
  private void acceptVision(VisionFieldPoseEstimate estimate) {drive.addVisionMeasurement(estimate);}
  public void stop() {drive.stop();}
  public RobotState getRobotState() {return state;}
  private void publishConfiguration() {
    java.util.ArrayList<String> missing=new java.util.ArrayList<>();
    for(var group:RobotSettings.class.getDeclaredClasses()) missing.addAll(RobotSettings.missing(group));
    SmartDashboard.putStringArray("Configuration/待設定",missing.toArray(String[]::new));
    SmartDashboard.putString("Vision/Configuration",RobotSettings.cameraReady()?"相機已設定":"視覺停用：參數未設定或 enabled=false");
    SmartDashboard.putString("Simulation/Configuration",RobotSettings.physicsReady()?"MapleSim NEO／NEO 550":"IO 測試替身：不是物理模擬");
  }
}
