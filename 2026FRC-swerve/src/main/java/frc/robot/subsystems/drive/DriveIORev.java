package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.MAXSwerveModule;

public final class DriveIORev implements DriveIO {
  private final MAXSwerveModule[] modules = {
      new MAXSwerveModule(DriveConstants.kFrontLeftDrivingCanId, DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset, DriveConstants.kFrontLeftDrivingInverted),
      new MAXSwerveModule(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset, DriveConstants.kFrontRightDrivingInverted),
      new MAXSwerveModule(DriveConstants.kRearLeftDrivingCanId, DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset, DriveConstants.kRearLeftDrivingInverted),
      new MAXSwerveModule(DriveConstants.kRearRightDrivingCanId, DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset, DriveConstants.kRearRightDrivingInverted)
  };
  @Override public void readInputs(DriveIOInputs inputs) {
    inputs.usable = true;
    for (int i = 0; i < modules.length; i++) inputs.usable &= modules[i].readInputs(inputs, i);
    inputs.status = inputs.usable ? "REV IO 正常" : "馬達初始化／編碼器讀取失敗；停止底盤輸出";
    //讀取四個輪組的資料：速度、角度、電壓、電流等，存進 inputs，供底盤計算、Elastic 顯示及記錄使用
    //檢查四輪是否都可用：全部正常，inputs.usable 才是 true；任何一輪回報失敗，就是 false，上層底盤程式會停止輸出
  }
  @Override public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < modules.length; i++) modules[i].setDesiredState(states[i]);
//把目標速度與角度交給四個輪組執行 意思就是 速度 ＝ 0.2 m/s    以 0.2 m/s 行走  馬達控制就由 MAXSwerveModule.setDesiredState() 處理
  }
  @Override public void stop() { for (var module : modules) module.stop(); }
}
