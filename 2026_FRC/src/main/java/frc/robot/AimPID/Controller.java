package frc.robot.AimPID;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class Controller {
    private final PIDController controller;

    public Controller() {
        this.controller = new PIDController(
            Constants.AimConstants.kAimP,
            Constants.AimConstants.kAimI,
            Constants.AimConstants.kAimD
        );
    }

    public double calculate(double err) {
        if (Math.abs(err) < Constants.AimConstants.kAllowErrDeg) {
            return 0.0;
        }

        double result = controller.calculate(err,0.0);
        if (result > 0.0) {
            result += Constants.AimConstants.kS;
        } else {
            result -= Constants.AimConstants.kS;
        }

        result /= Constants.DriveConstants.kHighAngularVelocityLimitRadPerSec;

        return MathUtil.clamp(result,-1.0 ,1.0);
    }
    public void reset() {
        controller.reset();
    }
}
