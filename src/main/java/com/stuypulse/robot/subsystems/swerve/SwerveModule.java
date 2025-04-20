package com.stuypulse.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Configs.SwerveModuleConfig;
import com.stuypulse.robot.constants.Settings.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final Rotation2d angleOffset;

    public SwerveModule(int driveId, int turnId, Rotation2d angleOffset) {
        driveMotor = new TalonFX(driveId, "*");
        turnMotor = new TalonFX(turnId, "*");

        driveMotor.getConfigurator().apply(SwerveModuleConfig.driveConfig);
        turnMotor.getConfigurator().apply(SwerveModuleConfig.turnConfig);

        this.angleOffset = angleOffset;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnMotor.getPosition().getValueAsDouble()).minus(angleOffset);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState.optimize(getAngle());
        desiredState.cosineScale(getAngle());

        driveMotor.setControl(new VelocityTorqueCurrentFOC(Units.radiansToRotations(desiredState.speedMetersPerSecond / Swerve.WHEEL_RADIUS)));
        turnMotor.setControl(new PositionTorqueCurrentFOC(desiredState.angle.getRotations()));
    }
}
