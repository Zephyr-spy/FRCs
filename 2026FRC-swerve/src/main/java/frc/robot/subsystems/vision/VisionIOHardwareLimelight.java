// Adapted from Team 254, FRC-2025-Public ae1aa582 (MIT).
// Source and local differences: docs/migration/TEAM254_VISION.md
package frc.robot.subsystems.vision;

import frc.robot.Constants.VisionConstants;
import frc.lib.limelight.LimelightHelpers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** 依 254 的 Limelight IO 讀取方式，改為本車單相機；只讀資料，不決定是否融合。 */
public class VisionIOHardwareLimelight implements VisionIO {
    protected final NetworkTable table =
            NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightTableName);

    private static final double[] DEFAULT_STDDEVS =
            new double[VisionConstants.kExpectedStdDevArrayLength];

    /** Creates a new Limelight vision IO instance. */
    public VisionIOHardwareLimelight() {
        setLLSettings();
    }

    /** Publishes the shared mounting configuration to the only Limelight. */
    private void setLLSettings() {
        // 啟動時由程式設定安裝位置；請修改 RobotSettings.Camera，而非只改網頁 UI。
        double[] cameraPose = {
            VisionConstants.kCameraForwardMeters,
            VisionConstants.kCameraRightMeters,
            VisionConstants.kCameraHeightMeters,
            frc.robot.config.RobotSettings.Camera.rollDegrees,
            VisionConstants.kCameraPitchDegrees,
            VisionConstants.kCameraYawDegrees
        };
        table.getEntry("camerapose_robotspace_set").setDoubleArray(cameraPose);
    }

    @Override
    public void readInputs(VisionIOInputs inputs) {
        readCameraData(table, inputs.camera, VisionConstants.kLimelightTableName);
    }

    /** Reads data from a single Limelight camera. */
    private void readCameraData(
            NetworkTable table, VisionIOInputs.CameraInputs camera, String limelightName) {
        camera.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
        camera.heartbeat = table.getEntry("hb").getDouble(Double.NaN);
        // 比 254 原版多做清空：失去目標時不能沿用上一輪的有效影格。
        camera.megatagPoseEstimate = null;
        camera.megatagCount = 0;
        camera.fiducialObservations = new FiducialObservation[0];
        camera.pose3d = null;
        camera.standardDeviations = table.getEntry("stddevs").getDoubleArray(DEFAULT_STDDEVS);

        // 沿用 254：tv 表示這一輪有看到有效目標，才讀取 MegaTag 1 定位。
        if (!camera.seesTarget) return;
        var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (mt1 != null && mt1.tagCount > 0) {
            camera.megatagPoseEstimate = MegatagPoseEstimate.fromLimelight(mt1);
            camera.megatagCount = mt1.tagCount;
            camera.fiducialObservations = FiducialObservation.fromLimelight(mt1.rawFiducials);
            camera.pose3d =
                    LimelightHelpers.toPose3D(LimelightHelpers.getBotPose_wpiBlue(limelightName));
        }
    }
}
