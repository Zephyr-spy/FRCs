package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.config.RobotSettings;

/** Pigeon2 只讀航向，不建立 CTRE 底盤。未確認安裝角度時保持不可信。 */
public final class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(RobotSettings.Gyro.canId, RobotSettings.Gyro.canBus);
  private boolean configured;
  private String configurationStatus = "待設定 Pigeon2 安裝 yaw/pitch/roll";

  public GyroIOPigeon2() {
    if (RobotSettings.complete(RobotSettings.Gyro.class)) {
      var config = new Pigeon2Configuration();
      config.MountPose.MountPoseYaw = RobotSettings.Gyro.yawDegrees;
      config.MountPose.MountPosePitch = RobotSettings.Gyro.pitchDegrees;
      config.MountPose.MountPoseRoll = RobotSettings.Gyro.rollDegrees;
      for (int attempt = 0; attempt < 3; attempt++) {
        var status = pigeon.getConfigurator().apply(config);
        configurationStatus = status.toString();
        if (status.isOK()) { configured = true; break; }
      }
    }
    BaseStatusSignal.setUpdateFrequencyForAll(50, pigeon.getYaw(), pigeon.getAngularVelocityZWorld());
  }
  @Override public void readInputs(GyroIOInputs inputs) {
    var yaw = pigeon.getYaw();
    var rate = pigeon.getAngularVelocityZWorld();
    var status = BaseStatusSignal.refreshAll(yaw, rate);
    inputs.yawRad = Math.toRadians(yaw.getValueAsDouble());
    inputs.yawRateRadPerSec = Math.toRadians(rate.getValueAsDouble());
    inputs.connected = status.isOK() && yaw.getTimestamp().getLatency() < 0.1
        && rate.getTimestamp().getLatency() < 0.1;
    inputs.trusted = configured && inputs.connected && Double.isFinite(inputs.yawRad)
        && Double.isFinite(inputs.yawRateRadPerSec);
    inputs.status = !configured ? configurationStatus : inputs.trusted ? "正常" : "Pigeon2 失聯或資料過期";
  }
}
