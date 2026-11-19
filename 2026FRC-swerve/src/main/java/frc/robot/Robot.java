package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.*;

/** Offseason lifecycle adapted for MAXSwerve. No hardware objects are created during replay. */
public class Robot extends LoggedRobot {
  private RobotContainer container;
  private Command autonomous;
  private static boolean replay;
  public static boolean isReplay() {return replay;}
  @Override public void robotInit() {
    replay=!RobotBase.isReal() && Boolean.getBoolean("frc.replay");
    Logger.recordMetadata("ProjectName","MAXSwerve-8805");
    Logger.recordMetadata("TemplateCommit","36d406b69a8df6648d1b3a40164f9e5abf2e2113");
    if(replay) {
      setUseTiming(false);
      String path=System.getProperty("frc.replayLog");
      if(path==null || path.isBlank())path=LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(path));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path,"_replay")));
    } else Logger.addDataReceiver(new WPILOGWriter());
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
    container=new RobotContainer();
  }
  @Override public void robotPeriodic() {CommandScheduler.getInstance().run();container.getRobotState().updateLogger();}
  private void cancelAndStop() {
    CommandScheduler.getInstance().cancelAll();
    if(autonomous!=null){autonomous.cancel();autonomous=null;}
    container.stop();
  }
  @Override public void disabledInit(){cancelAndStop();}
  @Override public void autonomousInit(){cancelAndStop();autonomous=container.getAutonomousCommand();autonomous.schedule();}
  @Override public void autonomousExit(){cancelAndStop();}
  @Override public void teleopInit(){cancelAndStop();}
  @Override public void teleopExit(){cancelAndStop();}
  @Override public void testInit(){cancelAndStop();}
  @Override public void testExit(){cancelAndStop();}
}
