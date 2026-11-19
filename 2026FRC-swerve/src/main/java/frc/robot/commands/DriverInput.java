package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

/** 純輸入轉換：先死區再倍率。不得在物理速度上另加模板的 5% 死區。 */
public final class DriverInput {
  private DriverInput() {}
  public static ChassisSpeeds speeds(double leftY,double leftX,double rightX) {
    return new ChassisSpeeds(axis(leftY)*OIConstants.kDriverSpeedScale*DriveConstants.kMaxSpeedMetersPerSecond,
        axis(leftX)*OIConstants.kDriverSpeedScale*DriveConstants.kMaxSpeedMetersPerSecond,
        axis(rightX)*OIConstants.kDriverSpeedScale*DriveConstants.kMaxAngularSpeed);
  }
  public static double axis(double value) {
    if (!Double.isFinite(value)) return 0;
    return -MathUtil.applyDeadband(MathUtil.clamp(value,-1,1),OIConstants.kDriveDeadband);
  }
}
