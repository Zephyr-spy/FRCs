package frc.lib.time;

import edu.wpi.first.wpilibj.Timer;

/** FPGA seconds on hardware; the recorded FPGA clock supplied by AdvantageKit in replay. */
public class RobotTime {
    public static double getTimestampSeconds() {
        return Timer.getTimestamp();
    }
}
