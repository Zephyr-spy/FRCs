package frc.robot.AimPID;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.DriverSubsystem;
import frc.robot.RobotContainer;

public class AimCommand extends Command { ;
    private final DriverSubsystem tank;
    private final Controller controller;
    private final CommandXboxController xboxController;
    private final SlewRateLimiter rateLimiter;

    public AimCommand(DriverSubsystem tank, CommandXboxController xboxController) {
        this.tank = tank;
        this.rateLimiter = new SlewRateLimiter(3.0);
        this.xboxController = xboxController;
        this.controller = new Controller();
        addRequirements(tank);
    }

    @Override
    public void execute() {
        double Aimturn = controller.calculate(tank.getTargetRotation2d().getDegrees());

        tank.drive(
            -Constants.DriveConstants.kNoramlLinearVelocityScale * rateLimiter.calculate(
            MathUtil.applyDeadband(
              xboxController.getLeftY(),
              Constants.OIConstants.kDeadZone
            )
        ), 
        -Aimturn, 
        true
        );

    } 

    @Override
    public void end(boolean interrupted) {
        tank.drive(0, 0);
        controller.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
