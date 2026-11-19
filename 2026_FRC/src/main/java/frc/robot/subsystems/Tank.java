package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class Tank {
    private final SparkMax LeaderMotor;
    private final SparkMax FollowMotor;
    
    private final RelativeEncoder encoder;
    
    private final SparkClosedLoopController controller;

    private final double d;

    public Tank(int leader, int follow, boolean invered){
        this.LeaderMotor = new SparkMax(leader, MotorType.kBrushless);
        this.FollowMotor = new SparkMax(follow, MotorType.kBrushless);
        
        final SparkMaxConfig followMotorConfig = new SparkMaxConfig();
        final SparkMaxConfig leaderMotorConfig = new SparkMaxConfig();

        leaderMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        
        leaderMotorConfig.encoder
            .positionConversionFactor(Constants.TankSideConstants.kDrivePositionConversionFactor)
            .velocityConversionFactor(Constants.TankSideConstants.kDriveVelocityConversionFactor);

        leaderMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1.0, 1.0)
            .p(Constants.TankSideConstants.kDriveP)
            .i(Constants.TankSideConstants.kDriveI)
            .d(Constants.TankSideConstants.kDriveD)
            .feedForward
                .kV(1.0 / Constants.TankSideConstants.kWheelFreeSpeedMps);

        followMotorConfig
            .idleMode(IdleMode.kBrake)
            .follow(leader, false);

        LeaderMotor.configure(leaderMotorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kNoPersistParameters
        );

        FollowMotor.configure(followMotorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kNoPersistParameters
        );

        this.encoder = LeaderMotor.getEncoder();
        this.controller = LeaderMotor.getClosedLoopController();

        if (invered) {
            this.d = -1.0;
        } else {
            this.d = 1.0;
        }

        resetEncoder();
    }

    public double getPosition() {
        return d * encoder.getPosition();
    }

    public double getVelocity() {
        return d * encoder.getVelocity();
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public void setVelocity(double velocity) {
        velocity *= d;
        controller.setSetpoint(velocity, ControlType.kVelocity);
    }
}
