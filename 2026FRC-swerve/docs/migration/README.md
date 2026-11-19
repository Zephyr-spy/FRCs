# offseason → MAXSwerve 移植流程確認

狀態：原始碼已實作；只完成 Python／文字靜態檢查，**尚未編譯、執行 Java 測試、模擬或部署**。本次也未 commit／push。必須先確認以下流程，才進行 Java 驗證。

視覺另依 [Team 254 版本與差異說明](TEAM254_VISION.md) 更新；該文件列出本次改動。

固定來源：PepsiLee/FRC-2026-Systemcore-Public-main，`offseason`，`36d406b69a8df6648d1b3a40164f9e5abf2e2113`。現有 `swerve` 歷史保留。套件維持 `frc.robot`，共用移植程式放 `frc.lib`。

## 1. 保留參數

開始實作時的本機原文與 SHA-256 保存於 `parameter-baseline.json`。`tools/check_migration.py` 比對既有常數、完整 REV Configs 與隊號設定，避免用更早聊天中的值覆蓋本機值。

| 項目 | 此次保留值／位置 |
|---|---|
| 隊號 | 8805；`.wpilib/wpilib_preferences.json` |
| 輪組順序 | 左前、右前、左後、右後（FL、FR、RL、RR） |
| 左前 drive／turn | 1／2；drive inverted=true |
| 右前 drive／turn | 3／4；drive inverted=false |
| 左後 drive／turn | 5／6；drive inverted=true |
| 右後 drive／turn | 7／8；drive inverted=false |
| 四輪安裝偏移 | 皆 0 rad，保留但仍需實機朝前校正確認 |
| 小齒輪、行走總減速比 | 12T、5.5:1 |
| 輪徑 | 0.0762 m |
| 輪組中心距 | 前後與左右皆 26.5 in＝0.6731 m |
| 底盤最大平移／旋轉 | 4.1 m/s、2π rad/s |
| 搖桿倍率 | 三軸共用 kDriverSpeedScale＝0.005（依後續指示合併） |
| 死區／手把 USB | 0.05／0 |
| 行走 PID／速度前饋 | 0.04、0、0／12 除以既有輪組理論自由速度 |
| 轉向 PID | 1、0、0，0～2π 位置環繞 |
| 行走／轉向電流限制 | 50 A／20 A，皆 Brake |
| 轉向編碼器 | Through Bore V2，讀值反轉，位置以 2π 換算 rad |
| NEO 自由轉速 | 5676 rpm |
| 原 S 型 Auto | 0.2 m/s、0.2 m/s²；X/Y/θ P 皆 1，角速度與角加速度皆 π |
| Pigeon2 | 新增，CAN ID 0、roboRIO CAN（`rio`），不使用 CANivore |

`Constants.java` 保留既有值，只有三個手把倍率合併為同值的共用設定；`Configs.java` 保留原檔內容。每個輪組建立獨立 drive 設定副本，維持可各別修改反轉。模擬與 PathPlanner 直接使用共用輪組幾何與 NEO 行走模型。

## 2. 開機與馬達流程

- `LoggedRobot` 啟動 AdvantageKit。真機建立 `DriveIORev` 與 `GyroIOPigeon2`，不建立 CTRE 底盤或 CTRE 馬達；取代原程式的 ADIS16470 SPI 陀螺儀。
- REV 仍套用原本重設／持久化設定流程，每顆初始化失敗最多嘗試三次；回傳結果公布到 Elastic。此流程**不是自動尋找四輪朝前零點**。絕對編碼器與目前零偏仍須人工確認。
- 四輪任一初始化、必要編碼器／電壓讀取失敗或有馬達故障，整個底盤停止並顯示原因；不讓剩下三輪繼續開。
- REV 的快取讀值與成功狀態不能單獨證明所有 CAN 資料即時更新；Elastic 狀態是診斷資訊，仍需實機確認第 8 顆與其他輪組的接線及編碼器。
- Pigeon 安裝 yaw／pitch／roll 未填，不寫入猜測的 MountPose，也不把航向當可信。填值且初始化成功、兩個訊號在 100 ms 內更新才允許航向功能。
- FPGA 秒統一用 WPILib 可 replay 的時間來源；共用 `SwerveDrivePoseEstimator` 融合輪組、Pigeon 與通過檢查的相機量測。Pigeon 恢復時保留目前估計姿態、重新建立歷史，避免突然套用航向跳變。失聯期間位置不可靠，需要重新確認場地定位。

## 3. 手動控制與按鍵

| 控制 | 行為 |
|---|---|
| 左 Y／左 X／右 X | 前後／橫移／旋轉，先死區，再共用倍率，再換算速度 |
| 全推前後或橫移 | 指令約 0.0205 m/s，沒有第二層速度死區；不代表克服摩擦後的實際速度 |
| 全推旋轉 | 指令約 0.03142 rad/s |
| Start | 保留位置 X/Y，將估計航向歸零；無可信 Pigeon 時拒絕並顯示原因 |
| RB 按住 | X 鎖輪；drive 速度為 0、turn 保持 X 角度 |
| X／Y | 未綁定；依 254 視覺定位流程移除額外追蹤命令 |

手動輸入與按鍵僅在 **Teleop Enabled＋手把連線** 時有效。RB 優先於 Start；Auto 不會被手把或按鍵接管。無可信航向時手动切為車身導向，不做紅藍方手把反向。

三軸倍率共用 `kDriverSpeedScale=0.005`，不再有前後／橫移／旋轉各自的加減速限制。Disabled、手把失聯、命令中斷與 RB 鎖輪立即停止。車頭維持開關仍預設關閉，啟用後的旋轉上限也使用共用手把倍率。

## 4. Auto 與 PathPlanner

- Auto Selector 預設「停止」，只有明確選取「S 型測試」才產生原先的 S 路徑（0,0 → 中間點 1,1／2,-1 → 3,0）。開始時重設里程計到軌跡起點；請確認實際擺車位置。
- Auto 需要可信航向與正常輪組。模式切換、條件失效或命令中斷時，停止 drive 與 turn 輸出。
- PathPlanner 使用模板本地 fork，另以 vendordep 固定支援依賴；實際 imports 為 `frc.lib.pathplanner`。只有 `Paths.enabled` 且質量、慣量、輪胎摩擦、保險桿尺寸、重心資料完整，才初始化模型及列出 deploy/pathplanner/autos 中的選項。
- 既有 Auto 約束也用於驗證 `.path` 全域與區域限制；超出就拒絕，請在 PathPlanner 降低限制。路徑控制回授輸出再限制至原 Auto 速度上限。路徑檔案格式版本字串 `2025.X` 是此固定版本函式庫的 JSON schema，**不是使用 2025 場地**。
- `DriveSubsystem` 建構子最後呼叫 `configurePathPlanner()` 設定模型及 `AutoBuilder`。路徑跟隨在底盤 `periodic()` 更新本輪定位後執行，使用同一個 20 ms 主循環；沒有獨立 PathPlanner 子系統或第二個寫馬達的控制執行緒。AD* 的背景執行緒只計算路線。
- `RobotContainer` 向底盤取得路徑選項及 Auto 命令；底盤 `stop()` 同時清除目前路徑與停止馬達，避免停止後下一輪又送出舊路徑。
- 尋路額外需要人工確認的 2026 navgrid、正確場地尺寸及有效障礙；預設未提供，完全停用尋路。沒有把空地圖當無障礙場地，也沒保留舊 Reef 障礙圖。
- 本次沒有預放 PathPlanner Auto／path／navgrid；也沒有登錄其他機構命令。

## 5. 相機、記錄與模擬

- 相機未填型號、數量、名稱、安裝位置角度及 yaw stddev 單位确认前，不建立實機相機 IO、不融合資料。此版僅接受一顆 Limelight。
- 配好並啟用後，程式啟動時將 `RobotSettings.Camera` 安裝位置寫至 Limelight；依 254 的 `processCamera()` 流程，以 MT1 為優先，單 Tag 可用歷史航向備援；只有 `tv=true` 才處理量測。過期、重複、未知 Tag、缺歷史或無可信航向會拒絕。
- 共同場地來自 WPILib `k2026RebuiltWelded`；定位、Tag 查表、Photon 模擬、路徑座標與翻轉尺寸共用它，不限制 2025 Reef Tag。
- `SmartDashboard/Swerve/...` 原八馬達欄位保留；新增初始化／命令回傳、底盤可用性、Pigeon 連線與可信度、功能停用原因。馬達 8 在目前參數是**右後轉向**。
- AdvantageKit 記錄 drive、gyro、完整相機輸入和 Auto 選擇；replay 不建硬體 IO。桌面 replay 可在確認後使用 `./gradlew simulateJava -Preplay -PreplayLog=/absolute/path/file.wpilog`。舊版沒有這些 inputs 的紀錄無法完整還原新架構。
- 物理設定未完成時只提供無輸出的 IO 測試替身，不能驗證機器人動態。填妥後 MapleSim 使用 NEO／NEO 550、12T／5.5 行走減速、現有輪徑及中心距，於主循環內每次跑 5×4 ms。
- 模型使用輸入的質量、慣量、保險桿尺寸、轉向減速、摩擦與轉向慣量；理想 gyro、不建熱模型。模擬已把輪組正方向視為校正完成，因此**不能驗證實際反轉、接線或零點**。
- 模擬內部障礙未確認，目前只有共用場地尺寸的外邊界；不能用它做避障或 2026 坡道碰撞驗證。Photon 的影像尺寸、FOV、FPS、延遲未填時不建立相機模擬；雜訊採明確理想模型，不冒充實機校正。

## 6. 待設定與警告

集中於 `src/main/java/frc/robot/config/RobotSettings.java`，目前 **33 項待設定／待確認**：

| 群組 | 缺項 | 目前影響 |
|---|---|---|
| Gyro（3） | Pigeon 安裝 yaw、pitch、roll | 手動車身導向；Auto／視覺停用 |
| Calibration（1） | 四輪朝前與零偏實機確認 | 顯示提醒，不自行改零偏 |
| Camera（10） | 型號、數量、NT 名稱、前後／左右／高度、三角度、yaw stddev 度單位確認 | 無實機相機 IO／融合 |
| Model（6） | 重量、垂直軸慣量、含保險桿長寬、重心高度、摩擦係數 | 無路徑模型、無物理模擬 |
| Physics（4） | 轉向減速、行走／轉向摩擦電壓、轉向慣量 | 僅 IO 測試替身 |
| CameraSim（5） | 寬高、對角 FOV、FPS、延遲 | 無 Photon 模擬 |
| Heading（3） | P/I/D | 開關預設關閉 |
| Paths（1） | navgrid 確認與實際障礙檔案 | 尋路停用 |

表格 Camera 項含 9 個參數＋1 個確認；總數依掃描器輸出為準。`Double.NaN`、空字串、-1 表示未設定；0 度是合法安裝角度，不能任意以 0 取代未知質量等參數。

VS Code：Terminal → Run Task → **檢查待設定參數**。Problems 中「機器人待設定」警告可點擊到宣告位置。補值後重跑會清除已完成的警告；功能是否啟用另外由各自 `enabled` 控制。此工作只執行 Python，不會跑 Gradle 或連接機器人。VS Code 的 Java 自動建置已關閉，Gradle 更新設為互動確認，以配合編譯前確認規則。

## 7. 驗證狀態與下一步確認範圍

已執行：`python3 tools/check_migration.py`、3 個參數警告 Python 測試、參數掃描及 `git diff --check`。基準值、REV 原檔、隊號、依賴、local imports 與禁止的硬體引用通過靜態核對；33 個警告是有意保留的未提供資料。

已寫好 **15 個但未執行** Java 測試案例：小倍率／死區、各輪反轉互不污染、Pigeon 失聯車身導向、Disabled／失聯／命令中斷停止、共用幾何、預設 Auto 停止、視覺過期與重複拒絕、單 Tag 歷史航向、MegaTag 優先、雙方向快速旋轉拒絕、tv=false、停用融合、未來／格式錯誤影格、replay 保留資料／清除舊影格。

請完整確認上面 1–6 的流程後，再授權桌面 `./gradlew build`（包含 Java 編譯和 JUnit）；模擬另行確認。首次編譯可能發現 API／依賴相容性問題，需依錯誤修正並重驗；目前不宣稱編譯通過。未完成物理參數時模擬只能驗證 IO／介面流程。

**此確認不包含部署、驅動實機馬達、commit 或 push。** 上傳與部署依後續明確指示處理。

## 8. 依賴與來源

| 依賴 | 固定版本 |
|---|---|
| WPILib / GradleRIO | 2026.2.1 |
| REVLib | 2026.0.5 |
| Phoenix 6 | 26.3.0（僅 Pigeon2） |
| AdvantageKit | 26.0.2 |
| PhotonLib | v2026.3.4 |
| MapleSim | 0.4.0-beta |
| PathPlanner vendordep | 2026.1.2；控制程式使用本地模板 fork |

來源、移植檔案範圍與授權見 `THIRD_PARTY.md`。
