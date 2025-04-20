package com.stuypulse.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class Configs {
    public static final class SwerveModuleConfig {
        public static final TalonFXConfiguration driveConfig;
        
        static {
            driveConfig = new TalonFXConfiguration();
        }

        public static final TalonFXConfiguration turnConfig;

        static {
            turnConfig = new TalonFXConfiguration();
        }
    }
}
