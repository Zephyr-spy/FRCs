// Adapted from Team 254, FRC-2025-Public ae1aa582 (MIT).
// Source and local differences: docs/migration/TEAM254_VISION.md
package frc.robot.subsystems.vision;

import frc.robot.Constants;
import frc.robot.config.RobotSettings.CameraSim;
import frc.robot.Constants.VisionConstants;
import frc.robot.simulation.SimulatedDriveState;
import frc.lib.limelight.LimelightHelpers;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * Simulation implementation of VisionIO using PhotonVision simulation. Extends Limelight hardware
 * implementation to reuse data processing logic.
 */
public class VisionIOSimPhoton extends VisionIOHardwareLimelight {
    private final PhotonCamera camera = new PhotonCamera(VisionConstants.kLimelightTableName);
    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim visionSim;
    private final SimulatedDriveState simulatedDriveState;

    private long heartbeat;

    private final int kResWidth = CameraSim.width;
    private final int kResHeight = CameraSim.height;

    /** Creates a new simulated vision IO instance using PhotonVision. */
    public VisionIOSimPhoton(SimulatedDriveState simulatedDriveState) {
        super();
        this.simulatedDriveState = simulatedDriveState;

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(Constants.kAprilTagLayout);

        SimCameraProperties prop = new SimCameraProperties();
        prop.setCalibration(kResWidth, kResHeight, Rotation2d.fromDegrees(CameraSim.diagonalFovDegrees));
        prop.setCalibError(0, 0); // 理想影像測試，未宣稱重現實機雜訊。
        prop.setFPS(CameraSim.fps);
        prop.setAvgLatencyMs(CameraSim.latencyMs);
        prop.setLatencyStdDevMs(0);
        prop.setExposureTimeMs(0);

        cameraSim = new PhotonCameraSim(camera, prop);
        cameraSim.setMinTargetAreaPixels(1); // 合成資料流測試，非實機可視範圍校正。

        // 使用與實機相同的安裝設定，只建立一顆模擬鏡頭。
        visionSim.addCamera(cameraSim, frc.robot.config.RobotSettings.cameraTransform());

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);
    }

    @Override
    public void readInputs(VisionIOInputs inputs) {
        Pose2d simulatedPose = simulatedDriveState.getLatestFieldToRobot();
        if (simulatedPose != null) {
            visionSim.update(simulatedPose);
            Logger.recordOutput("Vision/SimIO/updateSimPose", simulatedPose);
        }

        // 寫入父類別讀取的同一張表，沿用實機的單相機處理路徑。
        writeToTable(camera.getAllUnreadResults(), table, cameraSim);

        super.readInputs(inputs);
    }

    /** Generates robot pose data from PhotonVision results. */
    private List<Double> getBotpose(
            Transform3d fieldToCamera, PhotonPipelineResult result, PhotonCameraSim cameraSim) {
        if (result == null || !result.hasTargets()) return null;

        Optional<Transform3d> optRobotToCamera =
                visionSim.getRobotToCamera(cameraSim, Timer.getTimestamp());
        Pose3d fieldToRobot;
        if (optRobotToCamera.isPresent()) {
            Transform3d cameraToRobot = optRobotToCamera.get().inverse();
            Pose3d robotPose3d =
                    new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation())
                            .transformBy(cameraToRobot);
            fieldToRobot = robotPose3d;
        } else {
            fieldToRobot = new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation());
        }

        // metadata 的數量、平均距離及 raw fiducials 必須對應真正參與解算的標籤。
        var usedTargets =
                result.getMultiTagResult().isPresent()
                        ? result.getTargets().stream()
                                .filter(
                                        target ->
                                                result.getMultiTagResult()
                                                        .get()
                                                        .fiducialIDsUsed
                                                        .contains((short) target.getFiducialId()))
                                .toList()
                        : List.of(result.getBestTarget());
        double averageDistance =
                usedTargets.stream()
                        .mapToDouble(
                                target -> target.getBestCameraToTarget().getTranslation().getNorm())
                        .average()
                        .orElse(Double.NaN);
        List<Double> pose_data =
                new ArrayList<>(
                        Arrays.asList(
                                fieldToRobot.getX(),
                                fieldToRobot.getY(),
                                fieldToRobot.getZ(),
                                0.0,
                                0.0,
                                fieldToRobot.getRotation().getMeasureZ().in(Units.Degree),
                                result.metadata.getLatencyMillis(),
                                (double) usedTargets.size(),
                                0.0,
                                averageDistance,
                                result.getBestTarget().getArea()));

        for (var target : usedTargets) {
            pose_data.addAll(
                    Arrays.asList(
                            (double) target.getFiducialId(),
                            target.getYaw(), // txnc
                            target.getPitch(), // tync
                            target.getArea(), // ta
                            target.getBestCameraToTarget().getTranslation().getNorm(),
                            frc.robot.config.RobotSettings.cameraTransform()
                                    .plus(target.getBestCameraToTarget())
                                    .getTranslation()
                                    .getNorm(),
                            target.getPoseAmbiguity() // ambiguity
                            ));
        }
        return pose_data;
    }

    /**
     * Writes simulated vision data to NetworkTables for consumption by Limelight processing code.
     */
    private void writeToTable(
            List<PhotonPipelineResult> results, NetworkTable table, PhotonCameraSim cameraSim) {
        // 沒有新影格不等於失去標籤；保留原時間戳，VisionSubsystem 仍會檢查資料逾時。
        if (results.isEmpty()) return;
        for (var result : results) {
            List<Double> pose_data = null;
            if (result.getMultiTagResult().isPresent()) {
                var multiTagResult = result.getMultiTagResult().get();
                Transform3d best = multiTagResult.estimatedPose.best;

                pose_data = getBotpose(best, result, cameraSim);
            } else if (result.hasTargets()) {
                var bestTarget = result.getBestTarget();
                Transform3d best =
                        Constants.kAprilTagLayout
                                .getTagPose(bestTarget.getFiducialId())
                                .get()
                                .minus(Pose3d.kZero)
                                .plus(bestTarget.getBestCameraToTarget().inverse());

                pose_data = getBotpose(best, result, cameraSim);
            }

            long publishTimestamp =
                    Math.round(
                            (result.getTimestampSeconds()
                                            + result.metadata.getLatencyMillis() / 1000.0)
                                    * 1_000_000.0);
            table.getEntry("hb").setDouble(++heartbeat);
            if (pose_data != null) {
                double[] mt1 = pose_data.stream().mapToDouble(Double::doubleValue).toArray();
                LimelightHelpers.getLimelightDoubleArrayEntry(
                                VisionConstants.kLimelightTableName, "botpose_wpiblue")
                        .set(mt1, publishTimestamp);
                // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll,
                // MT2pitch,
                // MT2yaw]
                table.getEntry("stddevs")
                        .setDoubleArray(
                                new Double[] {
                                    0.3, 0.3, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                                });
            } else {
                table.getEntry("botpose_wpiblue").setDoubleArray(new double[0]);
            }
            table.getEntry("cl").setDouble(result.metadata.getLatencyMillis());
            table.getEntry("tl").setDouble(0.0); // cl 已包含 Photon 的總延遲。
            // 與實機 tv 相同：這一輪有有效定位目標；不依賴已移除的局部追蹤 ID。
            table.getEntry("tv").setDouble(pose_data != null ? 1.0 : 0.0);
        }
    }
}
