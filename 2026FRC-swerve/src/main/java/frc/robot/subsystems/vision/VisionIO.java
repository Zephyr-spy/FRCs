// Adapted from Team 254, FRC-2025-Public ae1aa582 (MIT).
// Source and local differences: docs/migration/TEAM254_VISION.md
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** 依 254 的分工：IO 只負責讀相機；是否採用這筆定位由 VisionSubsystem 決定。 */
public interface VisionIO {
    class VisionIOInputs implements LoggableInputs {
        /** 一顆相機這一輪讀到的資料；沒有解算結果時，estimate 和 pose3d 都是 null。 */
        public static class CameraInputs {
            public double heartbeat = Double.NaN;
            public boolean seesTarget;
            public FiducialObservation[] fiducialObservations = new FiducialObservation[0];
            public MegatagPoseEstimate megatagPoseEstimate;
            public int megatagCount;
            public Pose3d pose3d;
            // 前 6 項是 MT1 的 X/Y/Z/roll/pitch/yaw；後 6 項是 MT2。本車只使用 MT1。
            public double[] standardDeviations = new double[12];
        }

        /** 保留本車的單相機設定，未填參數時由 RobotContainer 使用空 IO。 */
        public final CameraInputs camera = new CameraInputs();

        /** 本車額外保留 AdvantageKit 回放；Tag ID 陣列另外記錄，避免可變長度資料遺失。 */
        @Override
        public void toLog(LogTable table) {
            var c = camera;
            table.put("Heartbeat", c.heartbeat);
            table.put("SeesTarget", c.seesTarget);
            table.put("Count", c.megatagCount);
            table.put("StdDevs", c.standardDeviations == null ? new double[0] : c.standardDeviations);
            table.put("Pose3d", c.pose3d == null ? new Pose3d[0] : new Pose3d[] {c.pose3d});
            var m = c.megatagPoseEstimate;
            table.put("Estimate", m == null ? new double[0] : new double[] {
                m.fieldToRobot().getX(), m.fieldToRobot().getY(), m.fieldToRobot().getRotation().getRadians(),
                m.timestampSeconds(), m.latency(), m.avgTagArea(), m.quality()
            });
            table.put("TagIds", m == null ? new int[0] : m.fiducialIds());
            // 同時是 record 與 StructSerializable，指定 struct 避免 LogTable 多載歧義。
            table.put("Fiducials", FiducialObservation.struct,
                    c.fiducialObservations == null ? new FiducialObservation[0] : c.fiducialObservations);
        }

        /** 回放每輪都覆蓋所有欄位；缺資料時清空，不沿用上一輪的有效影格。 */
        @Override
        public void fromLog(LogTable table) {
            var c = camera;
            c.heartbeat = table.get("Heartbeat", Double.NaN);
            c.seesTarget = table.get("SeesTarget", false);
            c.megatagCount = table.get("Count", 0);
            c.standardDeviations = table.get("StdDevs", new double[0]);
            Pose3d[] poses = table.get("Pose3d", new Pose3d[0]);
            c.pose3d = poses.length == 1 ? poses[0] : null;
            double[] m = table.get("Estimate", new double[0]);
            c.megatagPoseEstimate = m.length == 7 ? new MegatagPoseEstimate(
                    new Pose2d(m[0], m[1], Rotation2d.fromRadians(m[2])), m[3], m[4], m[5], m[6],
                    table.get("TagIds", new int[0])) : null;
            c.fiducialObservations = table.get("Fiducials", FiducialObservation.struct, new FiducialObservation[0]);
        }
    }

    void readInputs(VisionIOInputs inputs);
}
