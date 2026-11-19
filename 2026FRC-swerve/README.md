# 8805 MAXSwerve offseason 移植版

此專案已按固定 offseason 模板重構，保留 SPARK MAX／NEO／NEO 550 及原本参数。**目前尚未編譯驗證或部署。**

請先閱讀 [完整中文流程變更、保留參數與待設定清單](docs/migration/README.md)，確認後才編譯、執行 Java 測試或模擬。

視覺已依 [Team 254 的七個檔案分工與定位流程](docs/migration/TEAM254_VISION.md) 改編；來源、必要差異與本次操作變更列在該文件。手把三軸目前共用 `0.005` 倍率，X/Y 追蹤已移除。

- 修改既有硬體／手把參數：`src/main/java/frc/robot/Constants.java`、`Configs.java`。
- 新增功能與未知硬體資料：`src/main/java/frc/robot/config/RobotSettings.java`。
- VS Code 執行「檢查待設定參數」，只顯示中文 Problems，不編譯、不連機器人。
- Auto 預設停止；相機、路徑生成及物理模型缺值時停用。Pigeon 安裝未確認時使用車身導向。

以下保留原 REV 模板說明供來源參考；本版依賴版本與操作以中文移植文件為準。

---

# MAXSwerve Java Template v2026.0

See [the online changelog](https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/CHANGELOG.md) for information about updates to the template that may have been released since you created your project.

## Description

A template project for an FRC swerve drivetrain that uses REV MAXSwerve Modules.

Note that this template is designed for a drivetrain composed of four MAXSwerve Modules, each configured with two SPARKS MAX, a NEO as the driving motor, a NEO 550 as the turning motor, and a REV Through Bore Encoder as the absolute turning encoder. If you are using SPARK Flex for either the drive motor or turning motor, you will need to update the classes accordingly.

To get started, make sure you have calibrated the zero offsets for the absolute encoders in Hardware Client 2 using the `Absolute Encoder` utility under the associated turning SPARK devices.

## Prerequisites

* SPARK MAX Firmware v26.1.0
* REVLib v2026.0.0

## Configuration

It is possible that this project will not work for your robot right out of the box. Various things like the CAN IDs, PIDF gains, chassis configuration, etc. must be determined for your own robot!

These values can be adjusted in the `Configs.java` and `Constants.java` files.
