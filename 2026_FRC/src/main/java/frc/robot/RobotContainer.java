// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AimPID.AimCommand;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.IntakeSystem.IntakeStatus;
import frc.robot.subsystems.IntakeSystem.OPArmStatus;
import frc.robot.subsystems.ShooterSystem.ShooterStatus;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.concurrent.Delayed;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.DriverStation;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // private final DriverSubsystem m_driverSystem = new DriverSubsystem();
  private final IntakeSystem intake = new IntakeSystem();
  private final DriverSubsystem tank = new DriverSubsystem();
  private final SlewRateLimiter linearLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter angularLimiter = new SlewRateLimiter(2.0);
  
  private final SendableChooser<Command> autoChooser;
  private final ShooterSystem shooter = new ShooterSystem(
    () -> tank.getTargetDistanse()
  );
  
  private final CommandXboxController m_driverController = new CommandXboxController(0);  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("Aim", new AimCommand(tank, m_driverController));
    NamedCommands.registerCommand("Shooter", shooter.shooterCommand(ShooterStatus.SHOOT));
    // NamedCommands.registerCommand("Agitator", shooter.setAgitatorStatus(AgitatorStatus.RUN));
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> intake.setIntakeStatus(IntakeStatus.INTAKE)));
    NamedCommands.registerCommand("Arm", intake.setStatus(OPArmStatus.Down));
    // Configure the trigger bindings
    tank.setDefaultCommand(
      new RunCommand(
        () -> tank.drive(
          -Constants.DriveConstants.kNoramlLinearVelocityScale * linearLimiter.calculate(
            MathUtil.applyDeadband(
              m_driverController.getLeftY(),
              Constants.OIConstants.kDeadZone
            )
        ),
          -Constants.DriveConstants.kNoramlAngularVelocityScale * angularLimiter.calculate(
            MathUtil.applyDeadband(
              m_driverController.getRightX(),
              Constants.OIConstants.kDeadZone
            )
          )
      ),
      tank
    )
  );
  shooter.setDefaultCommand(shooter.shooterCommand(ShooterStatus.STOP));
  this.autoChooser = AutoBuilder.buildAutoChooser();
  SmartDashboard.putData("AutoChooser", autoChooser);
  


  configureBindings();
    }

  private void configureBindings(){
    //吸球
  m_driverController.leftTrigger().whileTrue(
    new StartEndCommand (
    () -> intake.setIntakeStatus(IntakeSystem.IntakeStatus.INTAKE),
    () -> intake.setIntakeStatus(IntakeSystem.IntakeStatus.INTAKESTOP),
    intake
    )

  );
    //按B吐球
    // m_driverController.y().whileTrue(
    //   m_intake.runEnd(
    //    ()-> m_intake.runIntake(Constants.IntakeConstants.kOutakeMotorSpeed),
    //    () -> m_intake.stopIntake())
    // );


    m_driverController.rightTrigger().whileTrue(
      shooter.shooterCommand(ShooterStatus.SHOOT)

    );
    // m_driverController.rightBumper().whileTrue(
    // shooter.setStatus(ShooterSystem.ShooterStatus.SLOW)
    // );

    m_driverController.pov(0).whileTrue(
      new StartEndCommand (
        () -> intake.setOPArmStatus(IntakeSystem.OPArmStatus.MANUAL),
        () -> intake.setOPArmStatus(IntakeSystem.OPArmStatus.ARMSTOP),
        intake
      )
    );
    m_driverController.pov(180).whileTrue(
      intake.setStatus(OPArmStatus.Down)
    );
    
    m_driverController.x().whileTrue(
      new AimCommand(tank, m_driverController)
    );

    m_driverController.a().onTrue(
      new InstantCommand(
        () -> {
          Pose2d point = null;
          var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
          if (alliance == DriverStation.Alliance.Blue) {
            point = new Pose2d(0.435,0.475,Rotation2d.kZero); // Blue
          } else {
            point = new Pose2d(16.105, 7.73, Rotation2d.k180deg); // Red
          }
          tank.resetOdometry(point);
        },
        tank
      )
    );

    }

  public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }
}
