package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

/**
 * 底盤與硬體之間的介面：定義「能讀哪些資料、能下哪些指令」。
 *
 * <p>這裡不直接控制馬達。實機由 DriveIORev 實作，模擬由 DriveIOMapleSim 實作，
 * DriveSubsystem 則使用這個介面讀資料與下指令。
 */
public interface DriveIO {
  // 讓 AdvantageKit 自動產生 DriveIOInputsAutoLogged，用來記錄與回放以下資料。
  @AutoLog
  class DriveIOInputs {
    // ── 四個輪組的資料 ──
    // double[] 表示小數陣列；new double[4] 建立四格，初始值都是 0.0。
    // 索引不是 CAN ID！[0] 左前、[1] 右前、[2] 左後、[3] 右後。
    // 各輪 CAN ID 由 Constants 設定，DriveIORev 按上述輪組順序讀取。

    
    public double[] positionsMeters = new double[4];
    /** 每輪目前的行走速度（公尺／秒），正負值表示行走方向。 */
    public double[] velocitiesMps = new double[4];
    /** 每輪相對車身的朝向（弧度），已扣除安裝偏移；0 朝前，正角度朝左。 */
    public double[] anglesRad = new double[4];
    /** 轉向絕對編碼器讀值（弧度），已套用編碼器設定，但尚未扣除輪組安裝偏移。 */
    public double[] absoluteAnglesRad = new double[4];
    /** 最近下達給每輪的目標行走速度（公尺／秒），不代表實際已達到。 */
    public double[] targetSpeedsMps = new double[4];
    /** 最近下達的目標轉向角度（弧度），與 absoluteAnglesRad 使用相同座標。 */
    public double[] targetAbsoluteRad = new double[4];
    /** 每輪是否已有控制目標；初始皆為 false。 */
    public boolean[] hasTarget = new boolean[4];

    // ── 八顆馬達的監測資料 ──
    // 每輪有 drive（行走）與 turn（轉向）兩顆馬達，所以需要八格。
    // [0] 左前 drive、[1] 左前 turn、[2] 右前 drive、[3] 右前 turn，
    // [4] 左後 drive、[5] 左後 turn、[6] 右後 drive、[7] 右後 turn。
    // 以下欄位只是存放資料；修改陣列數字不會直接控制馬達。

    /** 控制器的供電電壓（V）；實機讀 getBusVoltage()，馬達沒轉也可能有 12 V。 */
    public double[] voltage = new double[8];
    /** 控制器回報的馬達輸出比例（-1 到 1），不是電壓或轉速。 */
    public double[] output = new double[8];
    /** 控制器回報的馬達電流（A，安培）。 */
    public double[] current = new double[8];
    /** 控制器回報的溫度（°C，攝氏）。 */
    public double[] temperature = new double[8];
    /** 故障位元旗標；0 表示無回報故障，不是故障的數量。 */
    public int[] faults = new int[8];
    /** 警告位元旗標；0 表示無回報警告，不是警告的數量。 */
    public int[] warnings = new int[8];

    // ：供 Elastic 與 log 查看各步驟是否成功 ──
    // 同樣使用上述八顆馬達的順序；尚未讀取時不能只憑數字 0 判斷正常。

    /** 馬達控制器初始化／套用設定的結果。 */
    public String[] initStatus = emptyStatuses();
    /** 編碼器讀取結果：drive 為位置與速度的綜合結果，turn 為角度讀取結果。 */
    public String[] readStatus = emptyStatuses();
    /** 讀取供電電壓的結果。 */
    public String[] voltageReadStatus = emptyStatuses();
    /** 行走編碼器位置讀取結果；目前實機只填入 drive 的格子。 */
    public String[] positionReadStatus = emptyStatuses();
    /** 行走編碼器速度讀取結果；目前實機只填入 drive 的格子。 */
    public String[] velocityReadStatus = emptyStatuses();
    /** 最近一次馬達控制指令的結果。 */
    public String[] commandStatus = emptyStatuses();

    /** IO 判斷整組輪組資料是否可用；尚未取得資料時預設 false。 */
    public boolean usable = false;
    /** 整體 IO 狀態或不可用原因，供上層顯示。 */
    public String status = "未建立硬體：IO 測試替身，不是物理模擬";

    /** 建立八格文字陣列，每格先填「尚無資料」，避免留下 null。 */
    private static String[] emptyStatuses() {
      String[] values = new String[8];
      java.util.Arrays.fill(values, "尚無資料");
      return values;
    }
  }

  // ── 上層底盤可呼叫的三個操作 ──
  // default + 空的 {} 表示預設不做任何事。
  // DriveIORev／DriveIOMapleSim 會覆寫方法，執行實際硬體或模擬操作。

  /** 將目前讀到的輪組與馬達資料填入 inputs，供底盤計算、顯示與記錄。 */
  default void readInputs(DriveIOInputs inputs) {}

  /**
   * 下達四輪的目標：每個 SwerveModuleState 包含速度（公尺／秒）與朝向。
   * states 順序固定為左前、右前、左後、右後；朝向使用車身座標。
   */
  default void setModuleStates(SwerveModuleState[] states) {}

  /** 立即停止所有馬達輸出，不受手把加速度限制。 */
  default void stop() {}
}
