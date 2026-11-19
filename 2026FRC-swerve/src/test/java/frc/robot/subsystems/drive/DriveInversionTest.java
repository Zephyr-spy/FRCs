package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;
import frc.robot.Configs;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.MAXSwerveModule;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeAll;
import edu.wpi.first.hal.HAL;

class DriveInversionTest {
  @BeforeAll static void hal(){assertTrue(HAL.initialize(500,0));}
  @Test void constructingRightWheelCannotOverwriteLeftWheelOrCommonConfig() {
    String original=Configs.MAXSwerveModule.drivingConfig.flatten();
    var left=MAXSwerveModule.createDrivingConfig(DriveConstants.kFrontLeftDrivingInverted);
    String leftBefore=left.flatten();
    var right=MAXSwerveModule.createDrivingConfig(DriveConstants.kFrontRightDrivingInverted);
    assertNotSame(left,right);
    assertNotEquals(leftBefore,right.flatten());
    assertEquals(leftBefore,left.flatten());
    assertEquals(original,Configs.MAXSwerveModule.drivingConfig.flatten());
    assertEquals(leftBefore,MAXSwerveModule.createDrivingConfig(DriveConstants.kRearLeftDrivingInverted).flatten());
    assertEquals(right.flatten(),MAXSwerveModule.createDrivingConfig(DriveConstants.kRearRightDrivingInverted).flatten());
  }
}