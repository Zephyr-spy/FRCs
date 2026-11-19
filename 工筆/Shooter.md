# ShooterSystem

這個Code用來控制機器人的發射機構。它由三個主要部分組成：
1. **ShooterMotor**：提供高轉速把球射出去。
2. **ControlMotor**：兩顆馬達，在球準備好時，把球推進飛輪。
3. **AgitatorMotor**：防止卡球。

為了達到在任一位置皆可進球的目標，加入了**狀態機**與**距離對轉速**的設計。

---


### 1. Shooter距離對轉速
* **Code位置**：`this.map = new InterpolatingDoubleTreeMap(); map.put(2.0, 2500.0)...`
* **控制邏輯**：從斜拋運動我們知道，機器人離目標越遠，球需要的初速度（\(v_0\)）就要越大，所以使用 `InterpolatingDoubleTreeMap`。我們只需要實測並手動輸入幾個點（例如：2公尺時要2500轉、2.5公尺時要2640轉）。當機器人透過雷射或鏡頭量出一個介於中間的距離（例如 2.3 公尺）時，程式會使用斜率公式幫算出目標 RPM

### 2. 發射防呆
* **Code位置**：`if (Math.abs(getRPM() - map.get(...)) < 150.0)`
* **控制邏輯**：如果轉速還沒到，就強行把球送進去，球會射不遠、下墜，甚至導致馬達卡死，所以在 `periodic()` 裡面做了一個防呆機制。只有當ShooterMotor和目標轉速的誤差小於 150 RPM 時，ControlMotor和
AgitatorMotor才會啟動把球推過去。如果ShooterMotor還在加速，ControlMotor就會自動停止（`stopMotor`）。

### 3. 狀態機模式
* **code位置**：`public enum ShooterStatus { SHOOT, OUT, STOP }`
* **控制邏輯**：Shooter在不同狀況下，各個馬達要做不同的配合。所以把所有馬達的行為打包成三種**Status**。寫上層控制指令時，我們不需要分別去叫每一個馬達轉幾圈，只要輸入情況對應的狀態，程式就會自動去對應每個馬達該做的事。

### 4. 飛輪的滑行設定與高效能硬體同步
* **Code位置**：`shooterFConfig.idleMode(IdleMode.kCoast).follow(shooterLMotor, true);`
* **控制邏輯**：ShooterMotor在高速旋轉時擁有很大的動能。如果射完球突然給馬達煞車（Brake），齒輪箱和馬達會承受巨大的反向衝擊力而損壞，所以把飛輪馬達的 IdleMode 設為 `kCoast`（滑行）。當停止發射時，ShooterMotor會利用自身的慣性慢慢停下來。


