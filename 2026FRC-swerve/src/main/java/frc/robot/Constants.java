// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.config.RobotSettings;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.1;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = 0;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 8;

    // Independent drive motor direction: true = inverted, false = normal.
    public static final boolean kFrontLeftDrivingInverted = true;
    public static final boolean kFrontRightDrivingInverted = false;
    public static final boolean kRearLeftDrivingInverted = true;
    public static final boolean kRearRightDrivingInverted = false;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    // 死區之後套用的共用手把倍率：前後、橫移、旋轉一起調整。
    public static final double kDriverSpeedScale = 0.005;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
  // 場地定位、視覺模擬、路徑座標共用 2026 官方場地。
  public static final AprilTagFieldLayout kAprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final class VisionConstants {

        // Large variance used to downweight unreliable vision measurements
        public static final double kLargeVariance = 1e6;

        // Standard deviation constants
        public static final int kMegatag1XStdDevIndex = 0;
        public static final int kMegatag1YStdDevIndex = 1;
        public static final int kMegatag1YawStdDevIndex = 5;

        // Validation constants
        public static final int kExpectedStdDevArrayLength = 12;

        public static final int kMinFiducialCount = 1;

        public static final String kLimelightTableName = RobotSettings.Camera.tableName;
        public static final double kCameraForwardMeters = RobotSettings.Camera.forwardMeters;
        public static final double kCameraRightMeters = RobotSettings.Camera.rightMeters;
        public static final double kCameraHeightMeters = RobotSettings.Camera.heightMeters;
        public static final double kCameraPitchDegrees = RobotSettings.Camera.pitchDegrees;
        public static final double kCameraYawDegrees = RobotSettings.Camera.yawDegrees;

        // Team 254 MT1 主分支門檻；單 Tag 失敗時另試歷史航向備援。
        public static final double kDefaultAmbiguityThreshold = 0.19;
        public static final double kDefaultYawDiffThreshold = 5.0; // 度。
        public static final double kTagAreaThresholdForYawCheck = 2.0; // 影像面積百分比。
        public static final double kTagMinAreaForSingleTagMegatag = 1.0;
        public static final double kDefaultZThreshold = 0.2; // 公尺。
        public static final double kDefaultNormThreshold = 1.0; // 距離場地原點，非標籤距離。
        public static final double kHighYawLookbackSeconds = 0.3;
        public static final double kMaxVisionYawRateRadiansPerSecond = 5.0; // 僅限 gyro 備援。

        // 量測時間統一使用 FPGA 秒（replay 使用錄製時基），不做 CTRE 專用轉換。
        public static final double kMaxMeasurementAgeSeconds = 0.50;
        public static final double kFutureTimestampToleranceSeconds = 0.05;


    }

}
