package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriverSubsystem;
import frc.robot.subsystems.Tank;
public class DriverSubsystem extends SubsystemBase {
    private final Tank leftSide;
    private final Tank righSide;

    private final DifferentialDrivePoseEstimator estimator;
    
    private final Field2d field;

    private final AHRS gyro;

    private Pose2d simPose2d;
    private ChassisSpeeds lastChassisSpeeds;
    public DriverSubsystem() {
        this.field = new Field2d();
        SmartDashboard.putData("field", field);

        this.gyro = new AHRS(NavXComType.kMXP_SPI);
        gyro.reset();

        this.simPose2d = new Pose2d();
        this.lastChassisSpeeds = new ChassisSpeeds();

        this.leftSide = new Tank(
            Constants.DriveConstants.kLeftMotorPort,
            Constants.DriveConstants.kLeftFollowerMotorPort,
            false
        );

        this.righSide = new Tank(
            Constants.DriveConstants.kRightMotorPort,
            Constants.DriveConstants.kRightFollowerMotorPort,
            true
        );

        this.estimator = new DifferentialDrivePoseEstimator(
            Constants.DriveConstants.kDifferentialDriveKinematics,
            getRotation2d(), 
            leftSide.getPosition(),
            leftSide.getPosition(), new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.7, 0.7, 9999999)
        );

        RobotConfig config = null;
        try{      
            config = RobotConfig.fromGUISettings();
        }catch(Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetOdometry,
            this::getChassisSpeeds,
            (speeds, feedforwards) -> drive(speeds),
            new PPLTVController(0.02),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }


    public Rotation2d getRotation2d() {
        if(RobotBase.isReal()) {
            return gyro.getRotation2d();
        }else{
            return simPose2d.getRotation();

        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        if(RobotBase.isReal()) {
            return Constants.DriveConstants.kDifferentialDriveKinematics.toChassisSpeeds(
                new DifferentialDriveWheelSpeeds(
                    leftSide.getVelocity(),
                    righSide.getVelocity()
                    )
                );   
        }else{
            return this.lastChassisSpeeds;
        }
    }

    public DifferentialDriveWheelPositions getWheelPositions() {
        return new DifferentialDriveWheelPositions(
            leftSide.getPosition(), 
            righSide.getPosition()
        );
    }
    public Pose2d getPose() {
        if (RobotBase.isReal()) {
            return estimator.getEstimatedPosition();
        } else {
            return this.simPose2d;
        }
    }

    public Pose2d getTarget() {
        Pose2d target;
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
        if (alliance == DriverStation.Alliance.Blue) {
            if (getPose().getX()<4.7) {
                target = Constants.AimConstants.BLUE_POSE2D;
            } else {
                target = new Pose2d(
                    Constants.AimConstants.DELIVER_DIS,
                    getPose().getY(),
                    new Rotation2d()
                );
            }
        } else {
            if (getPose().getX()>11.8) {
                target = Constants.AimConstants.RED_POSE2D;
            } else {
                target = new Pose2d(
                    Constants.FieldConstants.kFieldLength - Constants.AimConstants.DELIVER_DIS,
                    getPose().getY(),
                    new Rotation2d()
                );
            }
        }
        return target;
    }

    public Transform2d getTargetTransform2d() {
        return getTarget().minus(getPose());
    }

    public double getTargetDistanse() {
        Transform2d transform2d = getTargetTransform2d();
        double x = transform2d.getX();
        double y = transform2d.getY();  
        return Math.sqrt(x*x + y*y);
    }

    public Rotation2d getTargetRotation2d() {
        Transform2d transform2d = getTargetTransform2d();
        Rotation2d targetRotation2d = Rotation2d.fromRadians(Math.atan2(transform2d.getY(), transform2d.getX()));
        return targetRotation2d;
    }

    public boolean isAimed() {
        return Math.abs(Math.tan(getTargetRotation2d().getRadians()) * getTargetDistanse()) < Constants.AimConstants.kAllowErrM;
    }

    /**
     * for not HighAngularVelocity
     * @param vx -1.0 ~ 1.0 will be transfer to true linear velocity
     * @param omega -1.0 ~ 1.0 will be transfer to true angular velocity
     */
    public void drive(double vx, double omega) {
        drive(vx, omega, false);
    }

    /**
     * 
     * @param vx -1.0 ~ 1.0 will be transfer to true linear velocity
     * @param omega -1.0 ~ 1.0 will be transfer to true angular velocity
     * @param isHighAngularVelocity if true the max angular velocity is 4*pi rad/s
     *  else the max angular velocity is 2*pi rad/s
     */
    public void drive(double vx, double omega, boolean isHighAngularVelocity) {
        double linearVelocity = vx * Constants.DriveConstants.kLinearVelocityLimitMps;
        double angularVelocity = omega;
        if (isHighAngularVelocity) {
            angularVelocity *= Constants.DriveConstants.kHighAngularVelocityLimitRadPerSec;
        } else {
            angularVelocity *= Constants.DriveConstants.kLowAngularVelocityLimitRadPerSec;
        }
        drive(new ChassisSpeeds(linearVelocity, 0.0, angularVelocity));
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.lastChassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        DifferentialDriveWheelSpeeds wheelSpeeds = 
            Constants.DriveConstants.kDifferentialDriveKinematics.toWheelSpeeds(chassisSpeeds);
        leftSide.setVelocity(wheelSpeeds.leftMetersPerSecond);
        righSide.setVelocity(wheelSpeeds.rightMetersPerSecond);

        SmartDashboard.putNumber("Real Speed", leftSide.getVelocity());
        SmartDashboard.putNumber("Target Speed", wheelSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("Speed Err", leftSide.getVelocity()-wheelSpeeds.leftMetersPerSecond);
    }

    public void zeroHeading() {
        gyro.reset();

        if (RobotBase.isSimulation()) {
            this.simPose2d = new Pose2d(simPose2d.getTranslation(), new Rotation2d());
        }
    }
    
    public void resetOdometry(Pose2d pose) {
        estimator.resetPosition(getRotation2d(), getWheelPositions(), pose);

        this.simPose2d = pose;
    }

    public void visionEstimate() {
        boolean hasTarget2D = LimelightHelpers.getTV("limelight");
        SmartDashboard.putBoolean("LL 2D HasTarget", hasTarget2D);
        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation(
            "limelight",
            estimator.getEstimatedPosition().getRotation().getDegrees(),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        );
        LimelightHelpers.PoseEstimate limelightEstimate = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (limelightEstimate == null) {
            SmartDashboard.putString("visionEstimate", "Null");
            return;
        }
        
        if (Math.abs(gyro.getRate()) > 360) {
            SmartDashboard.putString("visionEstimate", "Rate Out");
            doRejectUpdate = true;
        }

        if (limelightEstimate.tagCount == 0) {
            SmartDashboard.putString("visionEstimate", "No Tag");
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            SmartDashboard.putString("visionEstimate", "Success");
            estimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
            estimator.addVisionMeasurement(
                limelightEstimate.pose,
                limelightEstimate.timestampSeconds
            );
        }

        SmartDashboard.putBoolean("Reject", doRejectUpdate);
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            estimator.update(getRotation2d(), getWheelPositions());
            field.setRobotPose(estimator.getEstimatedPosition());
            visionEstimate();
        } else {
            field.setRobotPose(simPose2d);
        }
        SmartDashboard.putNumber("Left", getWheelPositions().leftMeters);
        SmartDashboard.putNumber("Right", getWheelPositions().rightMeters);
        SmartDashboard.putNumber("Aim Error", getTargetRotation2d().getDegrees());
        SmartDashboard.putNumber("Dis", getTargetDistanse());
    }

    @Override
    public void simulationPeriodic() {
        double dt = 0.02;
        
        Twist2d twist = new Twist2d(
            getChassisSpeeds().vxMetersPerSecond * dt,
            getChassisSpeeds().vyMetersPerSecond * dt,
            getChassisSpeeds().omegaRadiansPerSecond * dt
        );

        simPose2d = simPose2d.exp(twist);
    }
}
