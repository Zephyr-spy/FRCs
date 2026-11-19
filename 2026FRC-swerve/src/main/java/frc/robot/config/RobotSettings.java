package frc.robot.config;

import edu.wpi.first.math.geometry.*;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

/** 新增功能設定。NaN / 空字串 / -1 = 未設定；不要用猜測數字消除警告。 */
public final class RobotSettings {
  private RobotSettings() {}
  public static final class Gyro {
    public static final int canId = 0;
    public static final String canBus = "rio";
    @Pending("Pigeon2 安裝 yaw（度），依 Phoenix MountPose 定義；水平朝前可填 0")
    public static final double yawDegrees = Double.NaN;
    @Pending("Pigeon2 安裝 pitch（度），依 Phoenix MountPose 定義")
    public static final double pitchDegrees = Double.NaN;
    @Pending("Pigeon2 安裝 roll（度），依 Phoenix MountPose 定義")
    public static final double rollDegrees = Double.NaN;
  }
  public static final class Calibration {
    @Pending("四輪朝前時請確認目前 Constants 的四個零偏與 Through Bore V2 型號，確認後設 true")
    public static final boolean moduleCalibrationConfirmed = false;
  }
  public static final class Camera {
    public static final boolean enabled = false;
    @Pending("填相機型號；此 IO 支援 Limelight，其他型號需對應 IO")
    public static final String model = "";
    @Pending("填相機數量；本次單相機介面僅接受 1")
    public static final int count = -1;
    @Pending("Limelight NetworkTables 名稱，例如 limelight-front，必須與裝置一致")
    public static final String tableName = "";
    @Pending("鏡頭相對底盤中心前後位置（m），前方為正")
    public static final double forwardMeters = Double.NaN;
    @Pending("鏡頭相對底盤中心左右位置（m），右方為正，左方為負")
    public static final double rightMeters = Double.NaN;
    @Pending("鏡頭中心離地高度（m），需大於 0")
    public static final double heightMeters = Double.NaN;
    @Pending("鏡頭俯仰（度），抬頭為正")
    public static final double pitchDegrees = Double.NaN;
    @Pending("鏡頭水平朝向（度），0 朝前、90 朝左、180 朝後")
    public static final double yawDegrees = Double.NaN;
    @Pending("鏡頭側傾（度），依繞前向 X 軸右手定則")
    public static final double rollDegrees = Double.NaN;
    @Pending("Limelight stddevs[5] 的角度單位已核對為度，確認韌體文件後設 true")
    public static final boolean yawStdDevDegreesConfirmed = false;
  }
  public static final class Model {
    @Pending("含電池與保險桿整車重量（kg），需大於 0")
    public static final double massKg = Double.NaN;
    @Pending("整車繞垂直軸轉動慣量（kg·m²），需大於 0")
    public static final double moiKgMetersSquared = Double.NaN;
    @Pending("含保險桿前後外長（m），不可填輪組中心距")
    public static final double bumperLengthMeters = Double.NaN;
    @Pending("含保險桿左右外寬（m），不可填輪組中心距")
    public static final double bumperWidthMeters = Double.NaN;
    @Pending("重心離地高度（m），需大於 0")
    public static final double cogHeightMeters = Double.NaN;
    @Pending("輪胎／地面摩擦係數，需大於 0")
    public static final double wheelCOF = Double.NaN;
  }
  public static final class Physics {
    public static final boolean enabled = false;
    @Pending("MAXSwerve 轉向總減速比，需大於 0；不要填行走 5.5")
    public static final double steerReduction = Double.NaN;
    @Pending("行走模型摩擦電壓（V），可為 0")
    public static final double driveFrictionVolts = Double.NaN;
    @Pending("轉向模型摩擦電壓（V），可為 0")
    public static final double steerFrictionVolts = Double.NaN;
    @Pending("轉向機構轉動慣量（kg·m²），需大於 0")
    public static final double steerInertia = Double.NaN;
  }
  public static final class CameraSim {
    @Pending("模擬相機影像寬度（像素），需大於 0")
    public static final int width = -1;
    @Pending("模擬相機影像高度（像素），需大於 0")
    public static final int height = -1;
    @Pending("模擬相機對角視角（度），需介於 0 與 180")
    public static final double diagonalFovDegrees = Double.NaN;
    @Pending("模擬相機 FPS，需大於 0")
    public static final double fps = Double.NaN;
    @Pending("模擬相機平均延遲（ms），可為 0")
    public static final double latencyMs = Double.NaN;
  }
  public static final class Heading {
    public static final boolean enabled = false;
    @Pending("維持車頭方向 P（rad 誤差到 rad/s），需大於 0")
    public static final double p = Double.NaN;
    @Pending("維持車頭方向 I，不使用可填 0")
    public static final double i = Double.NaN;
    @Pending("維持車頭方向 D，不使用可填 0")
    public static final double d = Double.NaN;
  }
  public static final class Paths {
    public static final boolean enabled = false;
    @Pending("2026 navgrid 已核對障礙、場地尺寸與保險桿空間後設 true；檔案需放 deploy/pathplanner/navgrid.json")
    public static final boolean navgridConfirmed = false;
  }
  public static boolean positive(double v) { return Double.isFinite(v) && v > 0; }
  public static boolean complete(Class<?> type) { return missing(type).isEmpty(); }
  public static List<String> missing(Class<?> type) {
    var result = new ArrayList<String>();
    for (Field field : type.getDeclaredFields()) {
      Pending pending = field.getAnnotation(Pending.class);
      if (pending == null) continue;
      try {
        Object value = field.get(null);
        if ((value instanceof Double number && !Double.isFinite(number))
            || (value instanceof Integer integer && integer < 0)
            || (value instanceof String s && s.isBlank())
            || (value instanceof Boolean b && !b)) result.add(pending.value());
      } catch (IllegalAccessException e) { throw new IllegalStateException(e); }
    }
    return result;
  }
  public static boolean cameraReady() {
    return Camera.enabled && complete(Camera.class) && Camera.count == 1
        && Camera.model.toLowerCase(java.util.Locale.ROOT).contains("limelight")
        && positive(Camera.heightMeters);
  }
  public static boolean modelReady() {
    return complete(Model.class) && positive(Model.massKg) && positive(Model.moiKgMetersSquared)
        && positive(Model.bumperLengthMeters) && positive(Model.bumperWidthMeters)
        && positive(Model.cogHeightMeters) && positive(Model.wheelCOF);
  }
  public static boolean physicsReady() {
    return Physics.enabled && modelReady() && complete(Physics.class)
        && positive(Physics.steerReduction) && positive(Physics.steerInertia)
        && Physics.driveFrictionVolts >= 0 && Physics.steerFrictionVolts >= 0;
  }
  public static boolean cameraSimReady() {
    return cameraReady() && physicsReady() && complete(CameraSim.class)
        && CameraSim.width > 0 && CameraSim.height > 0 && positive(CameraSim.fps)
        && positive(CameraSim.diagonalFovDegrees) && CameraSim.diagonalFovDegrees < 180
        && CameraSim.latencyMs >= 0;
  }
  public static Transform3d cameraTransform() {
    if (!cameraReady()) throw new IllegalStateException("相機參數尚未設定或未啟用");
    return new Transform3d(new Translation3d(Camera.forwardMeters, -Camera.rightMeters, Camera.heightMeters),
        new Rotation3d(Math.toRadians(Camera.rollDegrees), -Math.toRadians(Camera.pitchDegrees), Math.toRadians(Camera.yawDegrees)));
  }
}
