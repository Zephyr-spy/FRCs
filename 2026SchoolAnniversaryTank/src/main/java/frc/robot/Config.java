package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Config {
    public static final class TankConfig {
        public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

        static {
            motorConfig
                .smartCurrentLimit(60)
                .idleMode(IdleMode.kBrake);
        }
    }

    public static final class ElevatorConfig {
        public static final SparkFlexConfig motorConfig = new SparkFlexConfig();

        static {
            motorConfig
                .smartCurrentLimit(80)
                .idleMode(IdleMode.kBrake)
                .inverted(false);
        }
    }
}
