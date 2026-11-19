package frc.robot.subsystems.tank;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TankConstants;

public class Tank extends SubsystemBase { 
    private final Side leftSide;
    private final Side rightSide;

    public Tank() {
        this.leftSide = new Side(
            TankConstants.kLeftFrontId,
            TankConstants.kLeftBackId,
            false
        );

        this.rightSide = new Side(
            TankConstants.kRightFrontId,
            TankConstants.kRightBackId,
            true
        );
    }

    private double bound(double x) {
        double y = 0;
        if (x > 0) {
            y = Math.min(x, 1.0);
        } else {
            y = Math.max(x, -1.0);
        }
        return y;
    }

    /**
     * 
     * @param ls linear speed
     * @param ts turn speed
     */
    public void drive(double ls, double ts) {
        ls *= TankConstants.kSpeedScale;
        ts *= TankConstants.kTurnSpeedScale;
        double leftSpeed = bound(ls + ts);
        double rightSpeed = bound(ls - ts);

        leftSide.set(leftSpeed);
        rightSide.set(rightSpeed);
    }

    public void stop() {
        leftSide.stop();
        rightSide.stop();
    }
}
