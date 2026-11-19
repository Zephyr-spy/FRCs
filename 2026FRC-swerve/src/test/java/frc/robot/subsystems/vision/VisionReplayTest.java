package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.*;
import org.littletonrobotics.junction.LogTable;

class VisionReplayTest {
  private final List<VisionFieldPoseEstimate> accepted = new ArrayList<>();
  private final VisionIO.VisionIOInputs recorded = new VisionIO.VisionIOInputs();
  private RobotState state;
  private VisionSubsystem vision;
  private double now = 10.05;

  @BeforeAll
  static void hal() { assertTrue(HAL.initialize(500, 0)); }

  @BeforeEach
  void setup() {
    state = new RobotState(accepted::add);
    state.setHeadingTrusted(true);
    state.addOdometryMeasurement(10, new Pose2d(2, 3, new Rotation2d()));
    var c = recorded.camera;
    c.seesTarget = true;
    c.megatagCount = 2;
    c.pose3d = new Pose3d(2, 3, 0, new Rotation3d());
    c.megatagPoseEstimate = new MegatagPoseEstimate(c.pose3d.toPose2d(), 10, .02, 4, 1, new int[] {1, 2});
    c.standardDeviations = new double[] {.2, .3, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0};
    c.fiducialObservations = new FiducialObservation[] {
        new FiducialObservation(1, 0, 0, .1, 4), new FiducialObservation(2, 0, 0, .1, 4)
    };
    vision = new VisionSubsystem(in -> {
      var table = new LogTable(10000000);
      recorded.toLog(table);
      in.fromLog(table);
    }, state, () -> now);
  }

  @AfterEach
  void cleanup() { CommandScheduler.getInstance().unregisterSubsystem(vision); }

  @Test
  void replayPreservesIdsAndRejectsDuplicateFrames() {
    vision.periodic();
    assertEquals(1, accepted.size());
    assertEquals(Math.toRadians(5), accepted.get(0).getVisionMeasurementStdDevs().get(2, 0), 1e-10);
    vision.periodic();
    assertEquals(1, accepted.size());
  }

  @Test
  void staleAndMissingHeadingCannotFuse() {
    now = 11;
    vision.periodic();
    assertTrue(accepted.isEmpty());
    now = 10.05;
    state.setHeadingTrusted(false);
    vision.periodic();
    assertTrue(accepted.isEmpty());
  }

  @Test
  void absentReplayFrameClearsOldEstimate() {
    var table = new LogTable(1);
    recorded.toLog(table);
    var restored = new VisionIO.VisionIOInputs();
    restored.fromLog(table);
    assertArrayEquals(new int[] {1, 2}, restored.camera.megatagPoseEstimate.fiducialIds());
    assertEquals(4, restored.camera.fiducialObservations[0].area());
    new VisionIO.VisionIOInputs().toLog(table);
    restored.fromLog(table);
    assertNull(restored.camera.megatagPoseEstimate);
    assertNull(restored.camera.pose3d);
    assertFalse(restored.camera.seesTarget);
    assertEquals(0, restored.camera.megatagCount);
    assertEquals(0, restored.camera.fiducialObservations.length);
  }

  private void singleTag(double area, double yawRate) {
    var tag = Constants.kAprilTagLayout.getTagPose(1).orElseThrow();
    var c = recorded.camera;
    c.megatagCount = 1;
    c.fiducialObservations = Arrays.copyOf(c.fiducialObservations, 1);
    c.megatagPoseEstimate = new MegatagPoseEstimate(
        new Pose2d(tag.getX() - 2, tag.getY(), new Rotation2d()), 10, .02, area, 1, new int[] {1});
    state.addOdometryMeasurement(10, new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
    state.addOdometryMeasurement(10.04, new Pose2d(2, 3, new Rotation2d()));
    for (double time : new double[] {9.69, 10, 10.04}) {
      state.addDriveMotionMeasurements(time, 0, 0, yawRate, 0, 0, 0, 0,
          new ChassisSpeeds(), new ChassisSpeeds(), new ChassisSpeeds(), new ChassisSpeeds(), new ChassisSpeeds());
    }
  }

  @Test
  void singleTagFallbackUsesCaptureTimeHeading() {
    singleTag(.5, 0); // 面積不足，MegaTag 主分支會拒絕。
    vision.periodic();
    assertEquals(1, accepted.size());
    var tag = Constants.kAprilTagLayout.getTagPose(1).orElseThrow();
    var estimate = accepted.get(0);
    assertEquals(tag.getX(), estimate.getVisionRobotPoseMeters().getX(), 1e-8);
    assertEquals(tag.getY() - 2, estimate.getVisionRobotPoseMeters().getY(), 1e-8);
    assertEquals(90, estimate.getVisionRobotPoseMeters().getRotation().getDegrees(), 1e-8);
    assertEquals(Constants.VisionConstants.kLargeVariance, estimate.getVisionMeasurementStdDevs().get(2, 0));
  }

  @Test
  void megatagWinsWhenBothSingleTagBranchesAreAvailable() {
    singleTag(4, 0);
    vision.periodic();
    assertEquals(1, accepted.size());
    assertEquals(recorded.camera.megatagPoseEstimate.fieldToRobot(), accepted.get(0).getVisionRobotPoseMeters());
    assertEquals(Math.toRadians(5), accepted.get(0).getVisionMeasurementStdDevs().get(2, 0), 1e-10);
  }

  @Test
  void missingTargetDoesNotReuseAnOtherwiseValidPose() {
    recorded.camera.seesTarget = false;
    vision.periodic();
    assertTrue(accepted.isEmpty());
  }

  @Test
  void disabledVisionDoesNotConsumeCandidate() {
    vision.setUseVision(false);
    vision.periodic();
    assertTrue(accepted.isEmpty());
    vision.setUseVision(true);
    vision.periodic();
    assertEquals(1, accepted.size());
  }

  @Test
  void clockwiseAndCounterclockwiseFastTurnsBothBlockFallback() {
    for (double yawRate : new double[] {-6, 6}) {
      singleTag(.5, yawRate);
      vision.periodic();
      assertTrue(accepted.isEmpty());
    }
  }

  @Test
  void futureAndMalformedFramesCannotFuse() {
    now = 9;
    vision.periodic();
    assertTrue(accepted.isEmpty());
    now = 10.05;
    recorded.camera.standardDeviations = new double[2];
    vision.periodic();
    assertTrue(accepted.isEmpty());
    recorded.camera.standardDeviations = new double[] {Double.NaN, .3, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0};
    vision.periodic();
    assertTrue(accepted.isEmpty());
  }
}