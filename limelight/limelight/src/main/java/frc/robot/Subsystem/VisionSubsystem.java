package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    
    private final String limelightName = "limelight";

    public VisionSubsystem() {
        LimelightHelpers.setPipelineIndex(limelightName, 0);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getVerticalOffset() {
        return LimelightHelpers.getTY(limelightName);
    }

    public double getTargetID() {
        return LimelightHelpers.getFiducialID(limelightName);
    }

    @Override
    public void periodic() {
        boolean seen = hasTarget();
        SmartDashboard.putBoolean("Vision/Has Target", seen);
        
        if (seen) {
            SmartDashboard.putNumber("Vision/TX (Horizontal)", getHorizontalOffset());
            SmartDashboard.putNumber("Vision/TY (Vertical)", getVerticalOffset());
            SmartDashboard.putNumber("Vision/Target ID", getTargetID());
        } else {
            SmartDashboard.putNumber("Vision/TX (Horizontal)", 0.0);
            SmartDashboard.putNumber("Vision/TY (Vertical)", 0.0);
        }
    }
}

