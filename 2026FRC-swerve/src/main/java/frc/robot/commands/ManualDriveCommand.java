package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.config.RobotSettings;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** 手動駕駛：三軸共用倍率；僅在 Teleop Enabled 且手把連線時送出要求。 */
public final class ManualDriveCommand extends Command {
  private final DriveSubsystem drive;
  private final BooleanSupplier enabled;
  private final DoubleSupplier y, x, rotation;
  private final PIDController heading;
  private Double heldHeading;

  public ManualDriveCommand(DriveSubsystem drive, BooleanSupplier enabled,
      DoubleSupplier y, DoubleSupplier x, DoubleSupplier rotation) {
    this.drive = drive;
    this.enabled = enabled;
    this.y = y;
    this.x = x;
    this.rotation = rotation;
    heading = RobotSettings.Heading.enabled && RobotSettings.complete(RobotSettings.Heading.class)
        && RobotSettings.positive(RobotSettings.Heading.p) && RobotSettings.Heading.i >= 0 && RobotSettings.Heading.d >= 0
        ? new PIDController(RobotSettings.Heading.p, RobotSettings.Heading.i, RobotSettings.Heading.d) : null;
    if (heading != null) heading.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    heldHeading = null;
    if (heading != null) heading.reset();
  }

  @Override
  public void execute() {
    if (!enabled.getAsBoolean()) {
      drive.stop();
      initialize();
      return;
    }
    double turn = rotation.getAsDouble();
    var requested = DriverInput.speeds(y.getAsDouble(), x.getAsDouble(), turn);
    if (heading != null && drive.hasHeading()) {
      // 用原始搖桿偵測轉向意圖，避免 0.005 倍率造成誤判。
      if (Math.abs(turn) > OIConstants.kDriveDeadband) {
        heldHeading = null;
        heading.reset();
      } else {
        if (heldHeading == null) heldHeading = drive.getPose().getRotation().getRadians();
        double max = DriveConstants.kMaxAngularSpeed * OIConstants.kDriverSpeedScale;
        requested.omegaRadiansPerSecond = MathUtil.clamp(
            heading.calculate(drive.getPose().getRotation().getRadians(), heldHeading), -max, max);
      }
    } else {
      heldHeading = null;
    }
    // 不再套用前後／橫移／旋轉各自的加減速度限制。
    drive.driveManual(requested.vxMetersPerSecond, requested.vyMetersPerSecond, requested.omegaRadiansPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    initialize();
  }
}
