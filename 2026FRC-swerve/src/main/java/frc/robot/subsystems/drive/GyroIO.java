package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;
    public boolean trusted = false;
    public double yawRad = 0;
    public double yawRateRadPerSec = 0;
    public String status = "尚無可靠航向";
  }
  default void readInputs(GyroIOInputs inputs) {}
}
