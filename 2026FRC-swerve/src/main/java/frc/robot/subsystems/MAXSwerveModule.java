// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Configs;
import com.revrobotics.REVLibError;
import frc.robot.subsystems.drive.DriveIO.DriveIOInputs;

public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private boolean m_hasTarget = false;
  private double m_driveTarget = 0;
  private double m_turnTarget = 0;
  private String driveInitStatus;
  private String turnInitStatus;
  private String driveCommandStatus = "尚未命令";
  private String turnCommandStatus = "尚未命令";

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset,
      boolean drivingInverted) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    // Copy the common settings so this module's direction cannot affect another module.
    SparkMaxConfig drivingConfig = createDrivingConfig(drivingInverted);
    driveInitStatus = configureMotor(m_drivingSpark, drivingConfig);
    turnInitStatus = configureMotor(m_turningSpark, Configs.MAXSwerveModule.turningConfig);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));
    m_hasTarget = true;
    m_driveTarget = correctedDesiredState.speedMetersPerSecond;
    m_turnTarget = correctedDesiredState.angle.getRadians();

    // Command driving and turning SPARKS towards their respective setpoints.
    driveCommandStatus = m_drivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity).name();
    turnCommandStatus = m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition).name();

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  /** Each call creates a new config; one wheel's inversion never mutates the shared template. */
  public static SparkMaxConfig createDrivingConfig(boolean drivingInverted) {
    SparkMaxConfig drivingConfig = new SparkMaxConfig();
    drivingConfig.apply(Configs.MAXSwerveModule.drivingConfig);
    drivingConfig.inverted(drivingInverted);
    return drivingConfig;
  }

  private static String configureMotor(SparkMax motor, SparkMaxConfig config) {
    String result = "未設定";
    for (int attempt = 0; attempt < 3; attempt++) {
      try {
        result = motor.configure(config, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters).name();
        if (result.equals("kOk")) break;
      } catch (IllegalStateException error) {
        result = error.getMessage() == null ? error.toString() : error.getMessage();
      }
    }
    return result;
  }

  public void stop() {
    m_drivingSpark.stopMotor();
    m_turningSpark.stopMotor();
    m_hasTarget = false;
    m_driveTarget = 0;
  }

  public boolean readInputs(DriveIOInputs inputs, int module) {
    int drive = module * 2, turn = drive + 1;
    inputs.positionsMeters[module] = m_drivingEncoder.getPosition();
    inputs.positionReadStatus[drive] = m_drivingSpark.getLastError().name();
    boolean valid = inputs.positionReadStatus[drive].equals("kOk");
    inputs.velocitiesMps[module] = m_drivingEncoder.getVelocity();
    inputs.velocityReadStatus[drive] = m_drivingSpark.getLastError().name();
    valid &= inputs.velocityReadStatus[drive].equals("kOk");
    inputs.readStatus[drive] = valid ? "kOk" : "行走編碼器讀取失敗";
    inputs.absoluteAnglesRad[module] = m_turningEncoder.getPosition();
    inputs.readStatus[turn] = m_turningSpark.getLastError().name();
    valid &= inputs.readStatus[turn].equals("kOk");
    inputs.anglesRad[module] = inputs.absoluteAnglesRad[module] - m_chassisAngularOffset;
    inputs.targetSpeedsMps[module] = m_driveTarget;
    inputs.targetAbsoluteRad[module] = m_turnTarget;
    inputs.hasTarget[module] = m_hasTarget;
    inputs.initStatus[drive] = driveInitStatus;
    inputs.initStatus[turn] = turnInitStatus;
    inputs.commandStatus[drive] = driveCommandStatus;
    inputs.commandStatus[turn] = turnCommandStatus;
    SparkMax[] motors = {m_drivingSpark, m_turningSpark};
    for (int i = 0; i < 2; i++) {
      var motor = motors[i]; int n = drive + i;
      inputs.voltage[n] = motor.getBusVoltage();
      inputs.voltageReadStatus[n] = motor.getLastError().name();
      valid &= inputs.voltageReadStatus[n].equals("kOk");
      inputs.output[n] = motor.getAppliedOutput();
      inputs.current[n] = motor.getOutputCurrent();
      inputs.temperature[n] = motor.getMotorTemperature();
      inputs.faults[n] = motor.getFaults().rawBits;
      valid &= motor.getLastError() == REVLibError.kOk;
      inputs.warnings[n] = motor.getWarnings().rawBits;
      valid &= inputs.faults[n] == 0;
    }
    return valid && driveInitStatus.equals("kOk") && turnInitStatus.equals("kOk")
        && Double.isFinite(inputs.positionsMeters[module])
        && Double.isFinite(inputs.velocitiesMps[module])
        && Double.isFinite(inputs.anglesRad[module]);
  }
}
