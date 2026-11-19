// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveConstants {
    public static final int kLeftMotorPort = 1;  //左主馬達腳位
    public static final int kLeftFollowerMotorPort = 2;  //左跟隨馬達腳位
    public static final int kRightMotorPort = 3;  //右主馬達腳位
    public static final int kRightFollowerMotorPort = 4;  //右跟隨馬達腳位
    public static final int kDriverControllerPort = 0;   //搖桿在電腦usb連接埠
    
    public static final double kTrackWidth = Units.inchesToMeters(20.5); // 0.5207 m
    public static final DifferentialDriveKinematics kDifferentialDriveKinematics = 
      new DifferentialDriveKinematics(kTrackWidth); 
    
    public static final double kFreeAngularVelocityRadPerSec = 
        TankSideConstants.kWheelFreeSpeedMps / (kTrackWidth / 2);

    public static final double kHighAngularVelocityLimitRadPerSec = 3.0 * Math.PI; // rad/s
    public static final double kLowAngularVelocityLimitRadPerSec = 2.0 * Math.PI; // rad/s
    public static final double kLinearVelocityLimitMps = 3.0; // m/s

    public static final double kNoramlLinearVelocityScale = 1.0;
    public static final double kNoramlAngularVelocityScale = 1.0;

        
  }
public static final class OIConstants {
      public static final double kDeadZone = 0.05;
    }
public static final class TankSideConstants {
    public static final double kGearRatio = (50.0 * 48.0) / (14.0 * 19.0);
    public static final double kWheelDiameter = Units.inchesToMeters(6.0); // 0.1524 m

    public static final double kDrivePositionConversionFactor = 
        (kWheelDiameter * Math.PI) / kGearRatio;
    public static final double kDriveVelocityConversionFactor = 
        kDrivePositionConversionFactor / 60.0;

    public static final double kWheelFreeSpeedMps = 
        MotorConstants.kNEO1650FreeSpeedRpm * kDriveVelocityConversionFactor;

        // non test 1-1
    public static final double kDriveP = 0.1;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
}
  public static final class MotorConstants {
    public static final double kNEO1650FreeSpeedRpm = 5676.0;
    public static final double kNEO550FreeSpeedRpm = 11000.0;
    public static final double kNEOVortexFreeSpeedRpm = 6784.0;
    public static final double kCimFreeSpeedRpm = 5310.0;
    }
  public static class AimConstants {
    public static final Pose2d BLUE_POSE2D = new Pose2d(4.625, 4.03, new Rotation2d());
    public static final Pose2d RED_POSE2D = new Pose2d(11.925, 4.03, new Rotation2d());

    public static final double DELIVER_DIS = 4.0;

    public static final double kAllowErrDeg = 0.25;
    public static final double kAllowErrM = 0.3;

    // 1-2
    public static final double kAimP = 0.12;
    public static final double kAimI = 0.0;
    public static final double kAimD = 0.0032;

    public static final double kS = 0.4;  
  }
  public static final class FieldConstants {
        public static final double kFieldLength = 16.5354;
        public static final double kFieldWidth = 8.001;
  }
  public static class IntakeConstants {
    public static final int kIntakeMotorId = 11  ;  //intake 
    public static final int kSensorDiochannel = 0;  //roborio Dio腳位
    public static final int kArmLd = 9;
    public static final int kArmFw = 10;

    public static final double kIntakeMotorSpeed = -1.0;  //intake 
    public static final double kOutakeMotorSpeed = 0.7;
    public static final double kArmUpSpeed = 0.25;
    public static final double kArmDownSpeed = -0.1;

  }


    
  
  public static class ShooterContants {
    public static final int kAgitator = 12;  //擾動馬達腳位
    public static final int kControlL = 7;  //主control馬達腳位
    public static final int kControlF = 8;  //跟隨control馬達腳位
    public static final int kshooterLeader = 5;  //主shooter
    public static final int kshooterFollow = 6;  //跟隨shooter

    public static final double kAgitatSpeed = 1.0;
    public static final double kControlLSpeed = 0.7;
    public static final double kHightSpeed = 1.0;
    public static final double kMediumSpeed = 0.7;
    public static final double kSlowSpeed = 0.5;
    public static final double kOutSpeed = -0.5;
    public static final double kAgitatOutSpeed = 0.0;
    public static final double kControlLOutSpeed = -0.7;

    
    }
}
