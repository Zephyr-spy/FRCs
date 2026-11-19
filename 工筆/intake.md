# IntakeSystem #

這個子系統負責控制機器人如何從場地上吸球以及如何透過Arm把intake放到定位。它由以下硬體組成：
1. **IntakeMotor**：負責吸球。
2. **ArmMotors**：由兩顆 Spark Max 無刷馬達帶動同一個旋轉軸，控制手臂的升降。

---


### 2. Intake
* **Code位置**：`intakeMotorConfig.idleMode(IdleMode.kCoast);`
* **控制邏輯**：吸完球關閉馬達時，intake通常還在高速旋轉，帶有動能，所以IntakeMotor我們設定為 `kCoast`（滑行模式）。

### 3. 使用狀態機與 Switch 架構
* **Code位置**：`public enum OPArmStatus { MANUAL, ARMSTOP, Down }` 與 `switch (currentOPArmStatus)`
* **控制邏輯**：設計了 `OPArmStatus` 狀態機，把手臂的運作鎖定在「停止、抬升、下降」這三種狀態，同一時間只會有一種狀態被選中。`periodic()` 會根據目前的狀態，去給予相對應的單一速度數值。

### 4. Command 的 runEnd 應用
* **Code位置**：`return runEnd(() -> this.setOPArmStatus(status), () -> this.setOPArmStatus(OPArmStatus.ARMSTOP));`
* **控制邏輯**：利用 WPILib 的 `runEnd` 控制方法。當按鍵按住時，執行前半段程式，一旦按鍵放開時，自動執行後半段程式。

