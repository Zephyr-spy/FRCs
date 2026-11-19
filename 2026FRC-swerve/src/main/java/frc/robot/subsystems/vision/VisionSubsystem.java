// Adapted from Team 254, FRC-2025-Public ae1aa582 (MIT).
// Source and local differences: docs/migration/TEAM254_VISION.md
package frc.robot.subsystems.vision;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.lib.time.RobotTime;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * 依 254 的流程處理 AprilTag 定位：MegaTag 1 優先，失敗時才嘗試單 Tag＋歷史航向。
 * 本車只有一個相機輸入；最後交給 RobotState 更新底盤定位，不在這裡控制馬達。
 */
public class VisionSubsystem extends SubsystemBase {
    private final VisionIO io;
    private final RobotState state;
    private final DoubleSupplier clock;
    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    private boolean useVision = true;

    public VisionSubsystem(VisionIO io, RobotState state) {
        this(io, state, RobotTime::getTimestampSeconds);
    }

    // 測試可指定時間；實機與 replay 使用 RobotTime 的 FPGA 秒。
    VisionSubsystem(VisionIO io, RobotState state, DoubleSupplier clock) {
        this.io = io;
        this.state = state;
        this.clock = clock;
    }

    @Override
    public void periodic() {
        double startTime = clock.getAsDouble();
        io.readInputs(inputs);
        Logger.processInputs("VisionInputs", inputs);
        logCameraInputs(inputs.camera);

        var accepted = processCamera(inputs.camera, startTime);
        if (useVision) {
            accepted.ifPresent(estimate -> {
                Logger.recordOutput("Vision/accepted", estimate.getVisionRobotPoseMeters());
                state.updateMegatagEstimate(estimate);
            });
        }
        Logger.recordOutput("Vision/usingVision", useVision);
        Logger.recordOutput("Vision/AcceptedThisCycle", useVision && accepted.isPresent());
        Logger.recordOutput("Vision/Status", !useVision ? "視覺停用：相機未設定或未啟用"
                : !state.isHeadingTrusted() ? "視覺停用：Pigeon 航向不可信"
                : accepted.isPresent() ? "本輪量測可用" : "本輪沒有可用的新量測");
        Logger.recordOutput("Vision/exclusiveTagId", state.getExclusiveTag().orElse(-1));
        Logger.recordOutput("Vision/latencyPeriodicSec", clock.getAsDouble() - startTime);
    }

    /** 254 的選擇順序：MegaTag 可用就採用；只有它失敗時，才採用 gyro 備援。 */
    private Optional<VisionFieldPoseEstimate> processCamera(
            VisionIO.VisionIOInputs.CameraInputs cam, double now) {
        Optional<VisionFieldPoseEstimate> mtEstimate = Optional.empty();
        Optional<VisionFieldPoseEstimate> gyroEstimate = Optional.empty();
        if (hasUsableFrame(cam, now)) {
            mtEstimate = processMegatagPoseEstimate(cam.megatagPoseEstimate, cam);
            gyroEstimate = fuseWithGyro(cam.megatagPoseEstimate, cam);
        }
        logCandidate("Vision/Camera/AcceptedMegatagEstimate", mtEstimate);
        logCandidate("Vision/Camera/FuseWithGyroEstimate", gyroEstimate);
        Logger.recordOutput("Vision/Camera/AcceptMegatag", mtEstimate.isPresent());
        Logger.recordOutput("Vision/Camera/AcceptGyro", mtEstimate.isEmpty() && gyroEstimate.isPresent());
        return mtEstimate.isPresent() ? mtEstimate : gyroEstimate;
    }

    /** 本車補上的輸入檢查：斷線、重複影格或格式不完整時，不把舊資料再次融入定位。 */
    private boolean hasUsableFrame(VisionIO.VisionIOInputs.CameraInputs cam, double now) {
        var estimate = cam.megatagPoseEstimate;
        if (!state.isHeadingTrusted() || !cam.seesTarget || estimate == null || cam.megatagCount < 1) {
            return false;
        }
        if (!Double.isFinite(now) || !Double.isFinite(estimate.timestampSeconds())
                || estimate.timestampSeconds() <= 0.0 || !Double.isFinite(estimate.latency())
                || estimate.latency() < 0.0) {
            return false;
        }
        double age = now - estimate.timestampSeconds();
        if (age < -VisionConstants.kFutureTimestampToleranceSeconds
                || age > VisionConstants.kMaxMeasurementAgeSeconds
                || estimate.timestampSeconds() <= state.lastUsedMegatagTimestamp()) {
            return false;
        }
        if (!finitePose(estimate.fieldToRobot()) || cam.pose3d == null || !Double.isFinite(cam.pose3d.getZ())) {
            return false;
        }
        if (estimate.fiducialIds().length != cam.megatagCount
                || Arrays.stream(estimate.fiducialIds()).anyMatch(id -> Constants.kAprilTagLayout.getTagPose(id).isEmpty())
                || cam.fiducialObservations == null || cam.fiducialObservations.length != cam.megatagCount
                || Arrays.stream(cam.fiducialObservations).anyMatch(tag -> tag == null || !Double.isFinite(tag.ambiguity()))) {
            return false;
        }
        return cam.standardDeviations != null
                && cam.standardDeviations.length == VisionConstants.kExpectedStdDevArrayLength
                && positiveFinite(cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex])
                && positiveFinite(cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex]);
    }

    /** MegaTag 主分支：沿用 254 的模糊度、面積、角度差、高度與品質篩選。 */
    private Optional<VisionFieldPoseEstimate> processMegatagPoseEstimate(
            MegatagPoseEstimate poseEstimate, VisionIO.VisionIOInputs.CameraInputs cam) {
        if (poseEstimate.fiducialIds().length < 2) {
            for (var fiducial : cam.fiducialObservations) {
                if (fiducial.ambiguity() > VisionConstants.kDefaultAmbiguityThreshold) {
                    return Optional.empty();
                }
            }
            if (poseEstimate.avgTagArea() < VisionConstants.kTagMinAreaForSingleTagMegatag) {
                return Optional.empty();
            }
            var priorPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
            if (poseEstimate.avgTagArea() < VisionConstants.kTagAreaThresholdForYawCheck
                    && priorPose.isPresent()) {
                double yawDiff =
                        Math.abs(
                                MathUtil.angleModulus(
                                        priorPose.get().getRotation().getRadians()
                                                - poseEstimate
                                                        .fieldToRobot()
                                                        .getRotation()
                                                        .getRadians()));
                if (yawDiff > Units.degreesToRadians(VisionConstants.kDefaultYawDiffThreshold)) {
                    return Optional.empty();
                }
            }
        }
        if (poseEstimate.fieldToRobot().getTranslation().getNorm()
                < VisionConstants.kDefaultNormThreshold) {
            return Optional.empty();
        }
        if (Math.abs(cam.pose3d.getZ()) > VisionConstants.kDefaultZThreshold) {
            return Optional.empty();
        }
        var exclusiveTag = state.getExclusiveTag();
        if (exclusiveTag.isPresent()
                && Arrays.stream(poseEstimate.fiducialIds())
                        .noneMatch(id -> id == exclusiveTag.get())) {
            return Optional.empty();
        }
        if (state.getFieldToRobot(poseEstimate.timestampSeconds()).isEmpty()) {
            return Optional.empty();
        }
        if (!positiveFinite(poseEstimate.quality())
                || poseEstimate.quality() > 1.0
                || !Double.isFinite(poseEstimate.avgTagArea())) {
            return Optional.empty();
        }
        // 沿用 254 的品質加權：品質越差，定位器越不相信這筆資料。
        double scaleFactor = 1.0 / poseEstimate.quality();
        double xStd = cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex] * scaleFactor;
        double yStd = cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex] * scaleFactor;
        // 本車的 WPILib 定位器使用 rad；Limelight yaw 標準差以度提供。
        double rotStd =
                Math.toRadians(cam.standardDeviations[VisionConstants.kMegatag1YawStdDevIndex]) * scaleFactor;
        double xyStd = Math.max(xStd, yStd);
        if (!positiveFinite(xyStd) || !positiveFinite(rotStd)) {
            return Optional.empty();
        }
        return Optional.of(
                new VisionFieldPoseEstimate(
                        poseEstimate.fieldToRobot(),
                        poseEstimate.timestampSeconds(),
                        VecBuilder.fill(xyStd, xyStd, rotStd),
                        poseEstimate.fiducialIds().length));
    }

    /** 單 Tag 備援：使用拍攝當時的航向修正 X/Y，給角度極大的標準差以保留 gyro 航向。 */
    private Optional<VisionFieldPoseEstimate> fuseWithGyro(
            MegatagPoseEstimate poseEstimate, VisionIO.VisionIOInputs.CameraInputs cam) {
        if (poseEstimate.fiducialIds().length != 1) {
            return Optional.empty();
        }
        double peakYawRate =
                state.getMaxAbsDriveYawAngularVelocityInRange(
                                poseEstimate.timestampSeconds()
                                        - VisionConstants.kHighYawLookbackSeconds,
                                poseEstimate.timestampSeconds())
                        .orElse(Double.POSITIVE_INFINITY);
        // 歷史查詢回傳有正負號的極值，因此兩個旋轉方向都要檢查絕對值。
        if (!Double.isFinite(peakYawRate)
                || Math.abs(peakYawRate) > VisionConstants.kMaxVisionYawRateRadiansPerSecond) {
            return Optional.empty();
        }
        var priorPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
        if (priorPose.isEmpty() || !finitePose(priorPose.get())) {
            return Optional.empty();
        }
        var maybeFieldToTag =
                Constants.kAprilTagLayout.getTagPose(poseEstimate.fiducialIds()[0]);
        if (maybeFieldToTag.isEmpty()) {
            return Optional.empty();
        }

        // 沿用 254 備援分支自己的篩選，不再套用 MegaTag 主分支的面積／角度差檢查。
        Pose2d fieldToTag =
                new Pose2d(maybeFieldToTag.get().toPose2d().getTranslation(), Rotation2d.kZero);
        Pose2d robotToTag = fieldToTag.relativeTo(poseEstimate.fieldToRobot());
        Pose2d posteriorPose =
                new Pose2d(
                        fieldToTag
                                .getTranslation()
                                .minus(
                                        robotToTag
                                                .getTranslation()
                                                .rotateBy(priorPose.get().getRotation())),
                        priorPose.get().getRotation());
        if (!finitePose(posteriorPose)) {
            return Optional.empty();
        }
        double xStd = cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex];
        double yStd = cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex];
        double xyStd = Math.max(xStd, yStd);

        return Optional.of(
                new VisionFieldPoseEstimate(
                        posteriorPose,
                        poseEstimate.timestampSeconds(),
                        VecBuilder.fill(xyStd, xyStd, VisionConstants.kLargeVariance),
                        poseEstimate.fiducialIds().length));
    }

    private static boolean positiveFinite(double value) {
        return Double.isFinite(value) && value > 0.0;
    }

    private static boolean finitePose(Pose2d pose) {
        return pose != null
                && Double.isFinite(pose.getX())
                && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getRotation().getRadians());
    }

    private void logCameraInputs(VisionIO.VisionIOInputs.CameraInputs cam) {
        Logger.recordOutput("Vision/Camera/Heartbeat", cam.heartbeat);
        Logger.recordOutput("Vision/Camera/SeesTarget", cam.seesTarget);
        Logger.recordOutput("Vision/Camera/MegatagCount", cam.megatagCount);
        Logger.recordOutput("Vision/Camera/Pose3d", cam.pose3d == null
                ? new Pose3d[0] : new Pose3d[] {cam.pose3d});
    }

    private static void logCandidate(String key, Optional<VisionFieldPoseEstimate> estimate) {
        Logger.recordOutput(
                key,
                estimate.map(value -> new Pose2d[] {value.getVisionRobotPoseMeters()})
                        .orElseGet(() -> new Pose2d[0]));
    }

    public void setUseVision(boolean useVision) {
        this.useVision = useVision;
    }
}
