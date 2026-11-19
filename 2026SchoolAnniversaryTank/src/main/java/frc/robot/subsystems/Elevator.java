package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config.ElevatorConfig;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final SparkFlex leftMotor;
    private final SparkFlex rightMotor;

    public Elevator() {
        this.leftMotor = new SparkFlex(
            ElevatorConstants.kLeftId,
            MotorType.kBrushless
        );
        this.rightMotor = new SparkFlex(
            ElevatorConstants.kRightId,
            MotorType.kBrushless
        );

        leftMotor.configure(
            ElevatorConfig.motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        rightMotor.configure(
            ElevatorConfig.motorConfig.follow(ElevatorConstants.kLeftId),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public void set(double x) {
        leftMotor.set(x);
    }

    public Command upCommand() {
        return runEnd(
            () -> this.set(ElevatorConstants.kUpSpeed),
            () -> this.stop()
        );
    }

    public Command downCommand() {
        return runEnd(
            () -> this.set(ElevatorConstants.kDownSpeed),
            () -> this.stop()
        );
    }
}
