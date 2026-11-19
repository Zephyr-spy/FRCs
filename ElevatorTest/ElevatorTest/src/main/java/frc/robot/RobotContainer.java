package frc.robot;

import main.java.frc.robot.ElevatorSubsystem;

public class RobotContainer {
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final CommandXboxController m_driverController = new CommandXboxController(0);  


    public RobotContainer(){
        m_driverController.a().whileTrue(
        elevator.run(() -> elevator.runElevator(Constants.ElevatorConstants.kElevatorUPSpeed))
                .finallyDo(elevator::stopElevator)
        );
    }
}
