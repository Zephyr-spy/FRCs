package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ManualDriveCommand;
import org.junit.jupiter.api.*;

class DriveControlTest {
  static class FakeDrive implements DriveIO {
    SwerveModuleState[] commanded;
    int stops;
    @Override public void readInputs(DriveIOInputs in){in.usable=true;}
    @Override public void setModuleStates(SwerveModuleState[] states){commanded=states;}
    @Override public void stop(){stops++;commanded=null;}
  }
  static class FakeGyro implements GyroIO {
    boolean valid=true;
    @Override public void readInputs(GyroIOInputs in){in.connected=valid;in.trusted=valid;in.yawRad=0;}
  }
  FakeDrive io;FakeGyro gyro;DriveSubsystem drive;
  @BeforeAll static void hal(){assertTrue(HAL.initialize(500,0));}
  @BeforeEach void setup(){
    DriverStationSim.resetData();DriverStationSim.setDsAttached(true);DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();DriverStation.refreshData();
    io=new FakeDrive();gyro=new FakeGyro();drive=new DriveSubsystem(io,gyro,new RobotState(v->{}));drive.periodic();
  }
  @AfterEach void cleanup(){CommandScheduler.getInstance().cancelAll();CommandScheduler.getInstance().unregisterSubsystem(drive);DriverStationSim.resetData();DriverStationSim.notifyNewData();}
  @Test void lostGyroFallsBackToRobotRelativeAndBlocksAuto(){
    drive.resetOdometry(new Pose2d(0,0,Rotation2d.fromDegrees(90)));
    drive.driveManual(.01,0,0);
    var field=DriveConstants.kDriveKinematics.toChassisSpeeds(io.commanded);
    assertEquals(-.01,field.vyMetersPerSecond,1e-8);
    gyro.valid=false;drive.periodic();assertFalse(drive.readyForAuto());
    drive.driveManual(.01,0,0);
    var body=DriveConstants.kDriveKinematics.toChassisSpeeds(io.commanded);
    assertEquals(.01,body.vxMetersPerSecond,1e-8);assertEquals(0,body.vyMetersPerSecond,1e-8);
  }
  @Test void disabledAndInterruptedManualStopImmediately(){
    boolean[] connected={true};
    var command=new ManualDriveCommand(drive,()->connected[0],()->-1,()->0,()->0);
    command.initialize();command.execute();assertNotNull(io.commanded);
    connected[0]=false;command.execute();assertNull(io.commanded);
    connected[0]=true;command.execute();command.end(true);assertNull(io.commanded);
    drive.driveManual(.01,0,0);assertNotNull(io.commanded);
    DriverStationSim.setEnabled(false);DriverStationSim.notifyNewData();DriverStation.refreshData();
    drive.periodic();assertNull(io.commanded);
  }
  @Test void sharedModuleGeometryMatchesKinematicsOrder(){
    var expected=DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,1));
    var locations=DriveSubsystem.moduleLocations();
    for(int i=0;i<4;i++) {
      assertEquals(-locations[i].getY(),expected[i].speedMetersPerSecond*expected[i].angle.getCos(),1e-10);
      assertEquals(locations[i].getX(),expected[i].speedMetersPerSecond*expected[i].angle.getSin(),1e-10);
    }
  }
}