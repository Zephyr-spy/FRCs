# 來源與授權

移植固定參考 [offseason / 36d406b](https://github.com/PepsiLee/FRC-2026-Systemcore-Public-main/tree/36d406b69a8df6648d1b3a40164f9e5abf2e2113)，不追蹤遠端新提交。

- `frc.lib.pathplanner`：模板的本地 PathPlanner fork（上游 Michael Jansen），保留根目錄 `PathPlanner-License.md`。此次改命名空間、場地尺寸入口、navgrid 缺檔拒絕與尋路啟用 gate；移除包含另一台車物理參數的 warmup。保留通用設定檔讀取器可辨識其他馬達名稱的能力，實際底盤模型只由 `DriveSubsystem.configurePathPlanner()` 建立 NEO 設定。
- `frc.lib.limelight`：模板提供的 LimelightHelpers，保留檔頭來源與授權註記。
- `frc.lib.time`、`frc.lib.util`、`RobotState`、`SimulatedDriveState`：改編 offseason 模板中的相同職責程式。模板 MIT 授權全文保存為 `Template-MIT-LICENSE.txt`（Copyright 2025 Team 254）。
- `frc.robot.subsystems.vision`：依使用者要求，直接對照 [Team254/FRC-2025-Public / ae1aa582](https://github.com/Team254/FRC-2025-Public/tree/ae1aa582b1cadb8462e6cb90718880d11a8f42b1/src/main/java/com/team254/frc2025/subsystems/vision) 的七個檔案改編。逐檔對照、保留流程與本車適配见 [TEAM254_VISION.md](TEAM254_VISION.md)。該專案 MIT 授權與已保存的 `Template-MIT-LICENSE.txt` 一致。「2025」是程式來源，不是本車場地配置。先前 offseason 的 `AprilTagObservation` 與追蹤命令已移除。
- `DriveIO`／REV IO／Pigeon IO／共用估計器／主循環控制／NEO 模擬接線：按本專案硬體重新實作。未複製模板的 CTRE 底盤設定、TalonFX/CANcoder 模擬或相機安裝值。
- 原 MAXSwerve 程式與 WPILib BSD 授權維持根目錄 `LICENSE.txt`；模板提供的 WPILib 授權另保存於 `Template-WPILib-License.md`。

二進位依賴（REV、Phoenix、AdvantageKit、PhotonLib、MapleSim）由 vendordep 依其上游授權管理；本文件未重新授權它們。參數基準檔是本機實作前內容，不是模板設定。

PathPlanner 接合程式由本專案重新實作；依使用者要求，初始化集中在 `DriveSubsystem` 建構子呼叫的 `configurePathPlanner()`，路徑更新由同一底盤的 `periodic()` 執行。這是組織方式調整，未改用官方 AutoBuilder 的另一套 API。
