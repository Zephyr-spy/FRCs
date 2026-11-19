package frc.robot.subsystems.tank;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Config.TankConfig;

public class Side {
    private final SparkMax frontMotor;
    private final SparkMax backMotor;

    public Side(int frontMotorId, int backMotorId, boolean inverted) {
        this.frontMotor = new SparkMax(frontMotorId, MotorType.kBrushless);
        this.backMotor = new SparkMax(backMotorId, MotorType.kBrushless);

        frontMotor.configure(
            TankConfig.motorConfig.inverted(inverted).follow(this.backMotor),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        backMotor.configure(
            TankConfig.motorConfig.inverted(inverted).disableFollowerMode(),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /**
     * stop motor
     */
    public void stop() {
        frontMotor.stopMotor();
        backMotor.stopMotor();
    }

    /**
     * set speed
     * @param speed -1.0 ~ 1.0
     */
    public void set(double speed) {
        backMotor.set(speed);
    }
}
