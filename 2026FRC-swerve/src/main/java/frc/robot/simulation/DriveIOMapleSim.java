package frc.robot.simulation;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.config.RobotSettings;
import frc.robot.subsystems.drive.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.*;
import org.ironmaple.simulation.drivesims.configs.*;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

/** NEO/NEO 550 mechanism model, stepped once in the main loop (5 × 4 ms).
 * 機構座標已校正正方向；此模型不能驗證實際馬達接線、反轉或 encoder 零點。
 * 僅模擬共用 2026 場地邊界，未確認內部障礙，不能用來驗證避障。
 */
public final class DriveIOMapleSim implements DriveIO {
  private final SwerveDriveSimulation sim;
  private final SimulatedArena arena;
  private final SimulatedDriveState truth;
  private final SwerveModuleState[] targets = new SwerveModuleState[4];
  private boolean active;
  private final double radius=ModuleConstants.kWheelDiameterMeters/2;

  private static final class BoundaryMap extends SimulatedArena.FieldMap {
    BoundaryMap() {
      double x=Constants.kAprilTagLayout.getFieldLength(),y=Constants.kAprilTagLayout.getFieldWidth();
      addBorderLine(new Translation2d(),new Translation2d(x,0));
      addBorderLine(new Translation2d(x,0),new Translation2d(x,y));
      addBorderLine(new Translation2d(x,y),new Translation2d(0,y));
      addBorderLine(new Translation2d(0,y),new Translation2d());
    }
  }
  private static final class BoundaryArena extends SimulatedArena {
    BoundaryArena(){super(new BoundaryMap());}
    @Override public void placeGamePiecesOnField() {}
  }
  public DriveIOMapleSim(SimulatedDriveState truth) {
    if(!RobotSettings.physicsReady())throw new IllegalStateException("物理參數尚未設定");
    this.truth=truth;
    SimulatedArena.overrideSimulationTimings(Seconds.of(.02),5);
    arena=new BoundaryArena();
    SimulatedArena.overrideInstance(arena);
    var config=DriveTrainSimulationConfig.Default()
        .withRobotMass(Kilograms.of(RobotSettings.Model.massKg))
        .withBumperSize(Meters.of(RobotSettings.Model.bumperLengthMeters),Meters.of(RobotSettings.Model.bumperWidthMeters))
        .withCustomModuleTranslations(DriveSubsystem.moduleLocations())
        // 理想航向用來測試資料流，不假裝已量測 Pigeon 的漂移／雜訊。
        .withGyro(()->new GyroSimulation(0,0))
        .withSwerveModule(new SwerveModuleSimulationConfig(DCMotor.getNEO(1),DCMotor.getNeo550(1),
            ModuleConstants.kDrivingMotorReduction,RobotSettings.Physics.steerReduction,
            Volts.of(RobotSettings.Physics.driveFrictionVolts),Volts.of(RobotSettings.Physics.steerFrictionVolts),
            Meters.of(radius),KilogramSquareMeters.of(RobotSettings.Physics.steerInertia),RobotSettings.Model.wheelCOF));
    // 合成測試起點置於邊界內，不代表實機初始定位。
    sim=new SwerveDriveSimulation(config,new Pose2d(Constants.kAprilTagLayout.getFieldLength()/2,
        Constants.kAprilTagLayout.getFieldWidth()/2,new Rotation2d()));
    sim.setMass(new org.dyn4j.geometry.Mass(sim.getMass().getCenter(),RobotSettings.Model.massKg,
        RobotSettings.Model.moiKgMetersSquared));
    arena.addDriveTrainSimulation(sim);
    for(int i=0;i<4;i++) {
      final int index=i;
      targets[i]=new SwerveModuleState();
      var driveLimiter=new GenericMotorController(DCMotor.getNEO(1)).withCurrentLimit(Amps.of(50));
      var turnLimiter=new GenericMotorController(DCMotor.getNeo550(1)).withCurrentLimit(Amps.of(20));
      sim.getModules()[i].useDriveMotorController((position,velocity,encoderPosition,encoderVelocity)-> {
        double target=active && DriverStation.isEnabled()?targets[index].speedMetersPerSecond:0;
        double volts=active && DriverStation.isEnabled()
            ?12*.04*(target-velocity.in(RadiansPerSecond)*radius)+12/ModuleConstants.kDriveWheelFreeSpeedRps*target:0;
        return driveLimiter.constrainOutputVoltage(encoderPosition,encoderVelocity,Volts.of(MathUtil.clamp(volts,-12,12)));
      });
      sim.getModules()[i].useSteerMotorController((position,velocity,encoderPosition,encoderVelocity)-> {
        double volts=active && DriverStation.isEnabled()
            ?12*MathUtil.angleModulus(targets[index].angle.getRadians()-position.in(Radians)):0;
        return turnLimiter.constrainOutputVoltage(encoderPosition,encoderVelocity,Volts.of(MathUtil.clamp(volts,-12,12)));
      });
    }
  }
  @Override public void readInputs(DriveIOInputs in) {
    if(!DriverStation.isEnabled())stop();
    arena.simulationPeriodic();
    truth.addFieldToRobot(sim.getSimulatedDriveTrainPose());
    in.usable=true;in.status="MapleSim NEO／NEO 550：僅場地邊界，未驗證內部障礙";
    for(int i=0;i<4;i++) {
      var module=sim.getModules()[i];
      in.positionsMeters[i]=module.getDriveWheelFinalPosition().in(Radians)*radius;
      in.velocitiesMps[i]=module.getCurrentState().speedMetersPerSecond;
      in.anglesRad[i]=module.getSteerAbsoluteFacing().getRadians();
      in.absoluteAnglesRad[i]=in.anglesRad[i];
      in.targetSpeedsMps[i]=targets[i].speedMetersPerSecond;
      in.targetAbsoluteRad[i]=targets[i].angle.getRadians();in.hasTarget[i]=active;
      in.current[2*i]=module.getDriveMotorSupplyCurrent().in(Amps);
      in.current[2*i+1]=module.getSteerMotorSupplyCurrent().in(Amps);
      in.output[2*i]=module.getDriveMotorAppliedVoltage().in(Volts)/12;
      in.output[2*i+1]=module.getSteerMotorAppliedVoltage().in(Volts)/12;
    }
    java.util.Arrays.fill(in.voltage,12);
    java.util.Arrays.fill(in.temperature,Double.NaN); // 無熱模型，不能顯示假溫度。
    java.util.Arrays.fill(in.initStatus,"SIM");java.util.Arrays.fill(in.readStatus,"SIM");
    java.util.Arrays.fill(in.commandStatus,"SIM");
    java.util.Arrays.fill(in.voltageReadStatus,"SIM");java.util.Arrays.fill(in.positionReadStatus,"SIM");
    java.util.Arrays.fill(in.velocityReadStatus,"SIM");
  }
  @Override public void setModuleStates(SwerveModuleState[] states) {
    for(int i=0;i<4;i++) {
      targets[i]=new SwerveModuleState(states[i].speedMetersPerSecond,states[i].angle);
      targets[i].optimize(sim.getModules()[i].getSteerAbsoluteFacing());
    }
    active=true;
  }
  @Override public void stop(){active=false;for(var target:targets)if(target!=null)target.speedMetersPerSecond=0;}
  public GyroIO gyroIO() {
    return new GyroIO(){@Override public void readInputs(GyroIOInputs in){
      in.connected=true;in.trusted=true;
      in.yawRad=sim.getGyroSimulation().getGyroReading().getRadians();
      in.yawRateRadPerSec=sim.getGyroSimulation().getMeasuredAngularVelocity().in(RadiansPerSecond);
      in.status="模擬理想航向；不是 Pigeon 實機校正";
    }};
  }
}
