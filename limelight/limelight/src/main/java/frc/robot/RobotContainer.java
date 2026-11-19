package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
// 1. 確保有導入您的子系統
import frc.robot.Subsystem.VisionSubsystem; 

public class RobotContainer {
  // 2. 宣告子系統變數
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  

  public RobotContainer() {
    m_visionSubsystem.setDefaultCommand(new InstantCommand());
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // 按鈕綁定...
  }
}
