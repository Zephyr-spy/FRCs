package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSystem.ShooterStatus;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSystem extends SubsystemBase {
    // 馬達硬體宣告
    private final SparkMax intakeMotor = new SparkMax(Constants.IntakeConstants.kIntakeMotorId, MotorType.kBrushless);
    private final SparkMax armFwMotor = new SparkMax(Constants.IntakeConstants.kArmFw, MotorType.kBrushless);
    private final SparkMax armLdMotor = new SparkMax(Constants.IntakeConstants.kArmLd, MotorType.kBrushless);

    // 專用狀態計時器
    private final Timer stateTimer = new Timer();

    // ===== 1. Intake 狀態機定義 =====
    public enum IntakeStatus {
        INTAKE(Constants.IntakeConstants.kIntakeMotorSpeed),
        INTAKESTOP(0.0);

        public final double speed;
        IntakeStatus(double speed) { this.speed = speed; }
    }

    // ===== 2. Arm 狀態機定義 =====
    // public enum ArmStatus {
    //     STOW(0.0, 0.0),                     // 靜止收回 (不給速，不計時)
    //     LOWER(Constants.IntakeConstants.kArmDownSpeed, 0.8);                  // 定時下降 (-0.2速度，跑1.2秒)
       
    //     public final double speed;
    //     public final double durationSeconds;

    //     ArmStatus(double speed, double durationSeconds) {
    //         this.speed = speed;
    //         this.durationSeconds = durationSeconds;
    //     }
    // }
    public enum OPArmStatus {
        MANUAL(Constants.IntakeConstants.kArmUpSpeed),
        ARMSTOP(0.0),
        Down(Constants.IntakeConstants.kArmDownSpeed);

        public final double armspeed;

        OPArmStatus(double armspeed){
            this.armspeed = armspeed;
        }
    }
    // 當前狀態暫存變數
    private IntakeStatus currentIntakeState = IntakeStatus.INTAKESTOP;
    // private ArmStatus currentArmState = ArmStatus.STOW;
    private OPArmStatus currentOPArmStatus = OPArmStatus.ARMSTOP;

    public IntakeSystem() {
        SparkMaxConfig armLdMotorConfig = new SparkMaxConfig();
        SparkMaxConfig armFwMotorConfig = new SparkMaxConfig();
        SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

        // 配置 Arm 主馬達
        armLdMotorConfig.inverted(true)
                        .idleMode(IdleMode.kBrake);

        // 配置 Arm 從馬達 (REV 2025/2026 新版 API 跟隨語法)
        armFwMotorConfig.follow(armLdMotor, false) 
                        .idleMode(IdleMode.kBrake);

        // 配置 Intake 馬達
        intakeMotorConfig.idleMode(IdleMode.kCoast);

        // 將配置寫入硬體
        armLdMotor.configure(armLdMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armFwMotor.configure(armFwMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // 啟動計時器
        stateTimer.start();
    }

    // ===== Intake 控制方法 =====
    public void setIntakeStatus(IntakeStatus status) {
        this.currentIntakeState = status;
    }

    public IntakeStatus getIntakeStatus() {
        return this.currentIntakeState;
    }

    // ===== Arm 狀態控制方法 =====
    // public void setArmStatus(ArmStatus status) {
    //     if (this.currentArmState != status) {
    //         this.currentArmState = status;
    //         stateTimer.reset(); // 關鍵：切換為定時狀態時重置計時器
    //     }
    // }

    // public ArmStatus getArmStatus() {
    //     return this.currentArmState;
    // }

    public void setOPArmStatus(OPArmStatus armStatus) {
        this.currentOPArmStatus = armStatus;
    }

    public OPArmStatus getOPArmStatus() {
        return this.currentOPArmStatus;
    }
    @Override
    public void periodic() {
        // ===== 執行 Intake 狀態機 =====
        switch (currentIntakeState) {
            case INTAKESTOP:
                intakeMotor.set(IntakeStatus.INTAKESTOP.speed);
                break;
            case INTAKE:
                intakeMotor.set(IntakeStatus.INTAKE.speed);
                break;
        }

        // ===== 執行 Arm 狀態機 =====
        // switch (currentArmState) {
        //     case STOW:
        //         armLdMotor.set(ArmStatus.STOW.speed);
        //         break;

        //     case LOWER:
        //         if (stateTimer.get() < ArmStatus.LOWER.durationSeconds) {
        //             armLdMotor.set(ArmStatus.LOWER.speed);
        //         } else {
        //             setArmStatus(ArmStatus.STOW); // 時間到，自動切回靜止
        //         }
        //         break;
        // }
        switch (currentOPArmStatus) {
            case ARMSTOP:
                armLdMotor.set(0.0);
                break;
        
            case MANUAL:
                armLdMotor.set(Constants.IntakeConstants.kArmUpSpeed);
                break;
            case Down:
                armLdMotor.set(Constants.IntakeConstants.kArmDownSpeed);
                break;
            default:
                armLdMotor.set(0.0);
        }
        
   


    }

    public Command setStatus(OPArmStatus status) {
        return runEnd(
            () -> this.setOPArmStatus(status),
            () -> this.setOPArmStatus(OPArmStatus.ARMSTOP)
        );
    }
}





