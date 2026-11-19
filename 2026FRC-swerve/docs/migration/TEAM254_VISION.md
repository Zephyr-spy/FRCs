# Team 254 視覺流程移植

狀態：已修改原始碼，尚未編譯、執行 Java 測試、模擬或部署。需先依使用者規則確認流程。

## 來源與本車差異

直接對照 [Team254/FRC-2025-Public 的 vision 資料夾](https://github.com/Team254/FRC-2025-Public/tree/ae1aa582b1cadb8462e6cb90718880d11a8f42b1/src/main/java/com/team254/frc2025/subsystems/vision)，固定 commit `ae1aa582b1cadb8462e6cb90718880d11a8f42b1`。授權為 MIT（Copyright 2025 Team 254），全文保存在 [Template-MIT-LICENSE.txt](Template-MIT-LICENSE.txt)。

保留 254 的七個類別名稱與職責；這是配合本車改編，並非原封不動複製。

| 檔案 | 職責 | 本車的必要差異 |
|---|---|---|
| `VisionIO.java` | 相機輸入介面及 CameraInputs | 雙相機改單相機；加入既有 AdvantageKit 輸入記錄／回放 |
| `VisionIOHardwareLimelight.java` | 讀取 tv、MegaTag 1、Tag 資訊、標準差；設定安裝位置 | 使用本車 Camera 設定；每輪清空失效資料；移除未使用的 RobotState 建構子參數與快取欄位 |
| `VisionIOSimPhoton.java` | Photon 模擬 → Limelight 格式 NT 資料 → 同一套 IO | 使用本車相機規格、安裝位置及 2026 場地；保留 Photon 拍攝時間與實際參與解算的 Tag 清單 |
| `FiducialObservation.java` | 一張 Tag 的識別、面積與模糊度 | 空值處理；Struct schema 包含實際儲存的 area |
| `MegatagPoseEstimate.java` | 相機估計的機器人位置、時間、品質、Tag ID | 延遲統一為秒；空值處理；修正 Struct 長度／schema 的 quality 欄位；可變長度 ID 由 VisionIO 另外記錄 |
| `VisionFieldPoseEstimate.java` | 通過篩選後交給定位器的姿態、時間與標準差 | 沿用資料結構，只改套件名稱與註解 |
| `VisionSubsystem.java` | 篩選、優先採用 MegaTag、單 Tag 航向備援、更新 RobotState | 單相機、不需要雙相機加權；保留 Pigeon 信任條件、時間／格式檢查、FPGA 時基及 rad 換算 |

`AprilTagObservation` 是先前 offseason 模板中的相機相對目標資料類別，不在這份 254 的 vision 資料夾內；本次已移除，連同使用它的 `AprilTagTrackingCommand` 與 X/Y 追蹤按鍵。

## 實際執行流程

1. `RobotContainer` 確認 Camera 設定完整且 `enabled=true`，才建立實機 Limelight IO。未設定時使用空 IO，不把猜測的相機安裝參數寫入裝置。
2. 每 20 ms 由 `VisionSubsystem.periodic()` 讀取 IO，先交給 AdvantageKit 記錄；replay 在這一步還原當時的輸入。
3. `processCamera()` 要求 `tv=true`，且量測格式完整、時間有效、非重複、Tag 存在於共用 2026 場地、Pigeon 航向可信。
4. `processMegatagPoseEstimate()` 沿用 254 主分支：單 Tag 檢查 ambiguity、面積與歷史航向差；另外檢查場地原點距離、Z、歷史資料、exclusive tag 與品質。標準差以品質加權，XY 取較大者。
5. 同時計算 `fuseWithGyro()` 候選，但**只有 MegaTag 主分支不接受，才使用這個候選**。必須只有一張 Tag、有拍攝當時的姿態，以及完整的 0.3 秒角速度歷史；旋轉速度絕對值超過 5 rad/s 就拒絕。
6. 備援以拍攝當時的歷史航向重算 X/Y，給航向很大的標準差；不再把相機航向融入底盤。保留 254 備援分支獨立的篩選方式。
7. 只有 `useVision=true` 才呼叫 `RobotState.updateMegatagEstimate()`，接到目前的 REV 底盤共用 `SwerveDrivePoseEstimator`。

本車補上的過期上限為 0.50 秒，未來時間容許值為 0.05 秒。角度標準差由度轉 rad；相機韌體單位仍需確認後才能啟用。這些是既有 MAXSwerve 接合需求，不宣稱是 254 原始碼全部都有的檢查。

## 本次會改到的操作

- 視覺只負責場地定位。X/Y 不再綁定「對準一次／按住跟隨」；RB 鎖輪、Start 歸零照原流程。
- 依先前指示，手把三軸共用 `Constants.OIConstants.kDriverSpeedScale = 0.005`，移除各軸加減速限制及其設定。死區仍為 0.05，全推平移指令仍為 0.0205 m/s，全推旋轉約 0.03142 rad/s。
- 車頭維持仍預設關閉；Auto 預設停止；Auto／PathPlanner 約束不屬於已移除的手把加速度設定，數值維持原樣。
- 馬達 CAN ID、獨立反轉、PID、12T、輪徑、中心距與八馬達監測維持本機基準。
- 場地仍為 `k2026RebuiltWelded`；不帶入 254 的 2025 Reef 白名單、兩顆相機安裝位置或 CTRE 底盤。
- `Vision/Camera/AcceptMegatag`、`AcceptGyro` 表示候選選擇；`Vision/AcceptedThisCycle` 表示是否實際交給定位流程。候選在停用融合時仍可供診斷。
- 刪除先前額外的 21 種 RejectionReason 列舉與追蹤欄位；以 `Vision/Status`、輸入紀錄、兩個候選與本輪接受旗標查看結果。
- 舊日誌中的 `Tracking` 欄位不再使用。既有相機定位欄位鍵名維持；新流程會遵守當時記錄的 `SeesTarget`。舊模擬曾把 tv 綁在追蹤目標，因此舊日誌重播結果可能與先前不同。

## 待設定與驗證

目前相機與 Pigeon 安裝角度仍未提供，視覺不會自動啟用。相機需填型號、數量、NT 名稱、前後／左右／高度、roll/pitch/yaw，並確認 stddevs 的角度單位。模擬還需要 CameraSim 規格與物理參數。

全專案待設定項目由 42 降為 33：移除了追蹤 6 項與手把加速度 3 項。詳細清單和完整啟動／控制流程見 [README.md](README.md)。

已完成 Python 參數核對、3 個參數警告測試及文字差異檢查。新增／更新的視覺 Java 測試涵蓋：MegaTag 優先、單 Tag 拍攝時航向、兩個方向的快速旋轉拒絕、tv=false、過期／重複／未來／格式錯誤影格、停用融合與回放清空。這些 Java 測試尚未執行。

待完整確認流程後，下一步僅執行桌面 `./gradlew build`（Java 編譯與 JUnit）；不部署、不驅動實機、不 commit／push。模擬須另行確認，且缺少物理參數時只能驗證 IO 流程。
