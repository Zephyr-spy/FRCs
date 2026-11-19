package frc.robot.simulation;

import frc.robot.RobotState;
import frc.lib.time.RobotTime;
import frc.lib.util.ConcurrentTimeInterpolatableBuffer;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;

/**
 * Shares simulated drivetrain truth with vision without depending on a robot container or game
 * pieces. The 20 ms main loop writes poses before the vision subsystem reads them.
 */
public final class SimulatedDriveState {
    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobotSimulatedTruth =
            ConcurrentTimeInterpolatableBuffer.createBuffer(RobotState.LOOKBACK_TIME);

    /** Records a drivetrain truth pose using the robot's current timestamp. */
    public void addFieldToRobot(Pose2d pose) {
        addFieldToRobot(RobotTime.getTimestampSeconds(), pose);
    }

    /** Records a truth pose with an explicit timestamp in seconds. */
    public void addFieldToRobot(double timestampSeconds, Pose2d pose) {
        fieldToRobotSimulatedTruth.addSample(timestampSeconds, pose);
    }

    /** Returns the latest truth pose, or null until the drive simulation publishes its first pose. */
    public Pose2d getLatestFieldToRobot() {
        var entry = fieldToRobotSimulatedTruth.getLatest();
        return entry == null ? null : entry.getValue();
    }

    /**
     * Samples retained truth history, interpolating between poses and clamping outside its bounds.
     * Returns an empty optional until the first pose is recorded.
     */
    public Optional<Pose2d> getFieldToRobot(double timestampSeconds) {
        return fieldToRobotSimulatedTruth.getSample(timestampSeconds);
    }
}
