package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ElevatorSubsystem extends SubsystemBase{
    private final SparkMax elevatorMotorLD = new SparkMax(Constants.ElevatorConstants.kElevatorLd, MotorType.kBrushless);
    private final SparkMax elevatorMotorFW = new SparkMax(Constants.ElevatorConstants.kElevatorFw, MotorType.kBrushless);
    public ElevatorSubsystem(){
        SparkMaxConfig elevatorLDConfig = new SparkMaxConfig();
        SparkMaxConfig elevatorFWConfig = new SparkMaxConfig();


        elevatorLDConfig.inverted(true)
                        .idleMode(IdleMode.kBrake);
        elevatorFWConfig.follow(Constants.ElevatorConstants.kElevatorLd)
                        .inverted(false)
                        .idleMode(IdleMode.kBrake);

        elevatorMotorLD.configure(elevatorLDConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotorFW.configure(elevatorFWConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }
    public void runElevator(double speed) {
        elevatorMotorLD.set(Constants.ElevatorConstants.kElevatorUPSpeed);
    }
    public void stopElevator(){
        elevatorMotorLD.set(0.0);
    }

    
}

