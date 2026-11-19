package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands; 

public class RobotContainer {
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final CommandXboxController m_driverController = new CommandXboxController(0);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        m_driverController.a().whileTrue(
            Commands.startEnd(
                () -> elevator.runElevator(Constants.ElevatorConstants.kElevatorUPSpeed),
                elevator::stopElevator,
                elevator
            )
        );
    }
    public Command getAutonomousCommand() {
    return null; 
}

}



