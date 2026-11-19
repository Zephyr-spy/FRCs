package frc.robot.subsystems;


import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSystem extends SubsystemBase{
   private final SparkFlex controlld = new SparkFlex(Constants.ShooterContants.kControlL,MotorType.kBrushless);
   private final SparkFlex controlfol = new SparkFlex(Constants.ShooterContants.kControlF,MotorType.kBrushless);
   private final SparkMax AgitatorMotor = new SparkMax(Constants.ShooterContants.kAgitator,MotorType.kBrushless);  
   private final SparkFlex shooterLMotor = new SparkFlex(Constants.ShooterContants.kshooterLeader,MotorType.kBrushless);
   private final SparkFlex shooterFMotor = new SparkFlex(Constants.ShooterContants.kshooterFollow,MotorType.kBrushless);

   private final SparkClosedLoopController controller;
   private final RelativeEncoder encoder;

   private final InterpolatingDoubleTreeMap map;
   private final DoubleSupplier disGetter;
   
   private ShooterStatus status;
   public enum ShooterStatus{
      SHOOT(
         Constants.ShooterContants.kControlLSpeed,
         Constants.ShooterContants.kAgitatOutSpeed
         
      ),
      OUT(
         Constants.ShooterContants.kControlLSpeed,
         Constants.ShooterContants.kAgitatOutSpeed
      ),
      STOP( 0, 0);

      
      private final double controlSpeed;
      private final double agitatorSpeed;

      ShooterStatus(double Controlspeed, double Agitatorspeed){
         this.controlSpeed = Controlspeed;
         this.agitatorSpeed = Agitatorspeed;
      }



      public double getAgitatorSpeed() {
          return agitatorSpeed;
      }

      public double getControlSpeed() {
          return controlSpeed;
      }



   }
   // private AgitatorStatus agitatorStatus;
   // public enum AgitatorStatus{
   //    RUN(Constants.ShooterContants.kAgitatSpeed),
   //    AGITATORSTOP(0.0);

   //    private final double agitatorSpeed;

   //    AgitatorStatus(double agitatorspeed){
   //       this.agitatorSpeed = agitatorspeed;
   //    }

   //    public double getAgitatorSpeed(){
   //       return agitatorSpeed;
   //    }
   // }
   public ShooterSystem(DoubleSupplier disGetter){
      SparkFlexConfig followConfig = new SparkFlexConfig();  //宣告shooter配置檔
      SparkFlexConfig controlConfig = new SparkFlexConfig();

      SparkFlexConfig shooterLConfig = new SparkFlexConfig();
      SparkFlexConfig shooterFConfig = new SparkFlexConfig();

      this.status = ShooterStatus.STOP;

      this.controller = this.shooterLMotor.getClosedLoopController();
      this.encoder = this.shooterLMotor.getEncoder();
      


      followConfig
         .idleMode(IdleMode.kBrake)
         .follow(controlld, true); 

      controlConfig
         .inverted(false)
         .idleMode(IdleMode.kBrake);



      shooterFConfig
         .idleMode(IdleMode.kCoast)
         .follow(shooterLMotor, true);

      shooterLConfig
         .inverted(false)
         .idleMode(IdleMode.kCoast);
      
      shooterLConfig.encoder
         .positionConversionFactor(1.0)
         .velocityConversionFactor(1.0);

         
         // 2-1
      shooterLConfig.closedLoop
         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
         .outputRange(-1.0, 1.0)
         .p(0.002)
         .i(0)
         .d(0.007)
         .feedForward
            .kV(1.06 / Constants.MotorConstants.kNEOVortexFreeSpeedRpm);


      
      controlfol.configure(
         followConfig, 
         ResetMode.kResetSafeParameters, 
         PersistMode.kPersistParameters
      );
      
      controlld.configure(
         controlConfig, 
         ResetMode.kResetSafeParameters, 
         PersistMode.kPersistParameters
      );



      shooterFMotor.configure(
         shooterFConfig,
         ResetMode.kResetSafeParameters,
         PersistMode.kPersistParameters
      );

      shooterLMotor.configure(
         shooterLConfig,
         ResetMode.kResetSafeParameters,
         PersistMode.kPersistParameters
      );

      resetEncoder();

      // 2-2
      this.map = new InterpolatingDoubleTreeMap();
      map.put(2.0, 2500.0);
      map.put(2.5, 2640.0);
      map.put(3.0, 3080.0);
      map.put(3.5, 3160.0);
      map.put(4.0, 3570.0);
      map.put(4.5, 3670.0);
      map.put(5.0, 3710.0);
      map.put(5.5, 3760.0);

      this.disGetter = disGetter;
   }
 
   /**
    * 
    * @param Agitatorspeed 1.0 ~ -1.0
    * @param Controlspeed 1.0 ~ -1.0
    */
   public void runControl( double Controlspeed, double Agitatorspeed){
      AgitatorMotor.set(Agitatorspeed);  //擾動速度
      controlld.set(Controlspeed);  //Shooter速度  
   }

   /**
    * 
    * @param Shooterspeed 1.0 ~ -1.0
    */
   public void setShooter(ShooterStatus status){
      this.status = status;
   }

   public ShooterStatus getShooter(){
      return this.status;
   }
   // public void setAgitator(AgitatorStatus agitatorStatus){
   //    this.agitatorStatus = agitatorStatus;
   // }

   // public ShooterStatus getAgitatorStatus(){
   //    return this.agitatorStatus;
   // }
   
   public void stopAll() {
      AgitatorMotor.set(0.0);
      controlld.set(0.0);
   }

   public void setRPM(double rpm) {
      controller.setSetpoint(rpm, ControlType.kVelocity);
   }

   public double getRPM() {
      return encoder.getVelocity();
   }

   public void resetEncoder() {
      encoder.setPosition(0.0);
   }

   @Override
   public void periodic(){
      SmartDashboard.putNumber("RPM Err", getRPM() - map.get(disGetter.getAsDouble()));
      SmartDashboard.putNumber("Real RPM", getRPM());
      SmartDashboard.putNumber("Target RPM", map.get(disGetter.getAsDouble()));
      


      if (this.status == ShooterStatus.SHOOT) {
         setRPM(map.get(disGetter.getAsDouble()));
         if (Math.abs(getRPM() - map.get(disGetter.getAsDouble())) < 150.0) {
            controlld.set(this.status.getControlSpeed());
            AgitatorMotor.set(this.status.getAgitatorSpeed());

         }else{
            controlld.stopMotor();
            AgitatorMotor.stopMotor();
         }
      } else {
         shooterLMotor.stopMotor();
         controlld.set(this.status.getControlSpeed());
         AgitatorMotor.set(this.status.getAgitatorSpeed());
      }
     

      // if (this.agitatorStatus == AgitatorStatus.RUN){
      //    AgitatorMotor.set(AgitatorStatus.RUN.getAgitatorSpeed());
      // }else{
      //    AgitatorMotor.set(0.0);
      // }
   }
   
   public Command shooterCommand(ShooterStatus status) {
      return this.runEnd(
        () -> this.setShooter(status), 
         () -> this.setShooter(ShooterStatus.STOP)
        
        
        );
   }
   // public Command setStatus(ShooterStatus status) {
   //    return runEnd(
   //       () -> this.setShooter(status),
   //       () -> this.setShooter(ShooterStatus.STOP)
   //    );
   // }
   // public Command setAgitatorStatus(AgitatorStatus agitatorStatus) {
   //    return runEnd(
   //       () -> this.agitatorStatus = AgitatorStatus.RUN, 
   //       () -> this.agitatorStatus = AgitatorStatus.AGITATORSTOP
   //    );
   // }


}
