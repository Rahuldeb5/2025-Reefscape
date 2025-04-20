package com.stuypulse.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    public final static Drive instance;
    
    static {
        instance = new Drive(
            new SwerveModule[4]
        );
    }

    public Drive getInstance() {
        return instance;
    }

    public SwerveModule[] modules;

    public Drive(SwerveModule... modules) {
        this.modules = modules;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("FL Angle (Rotations)", modules[0].getAngle().getRotations());
        SmartDashboard.putNumber("FR Angle (Rotations)", modules[0].getAngle().getRotations());
        SmartDashboard.putNumber("BL Angle (Rotations)", modules[0].getAngle().getRotations());
        SmartDashboard.putNumber("BR Angle (Rotations)", modules[0].getAngle().getRotations());
    }
}
