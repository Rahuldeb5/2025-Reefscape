package com.stuypulse.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors.SwerveModuleConfig;
import com.stuypulse.robot.constants.Settings.Swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final Rotation2d angleOffset;

    private SimpleMotorFeedforward ffModel;

    public SwerveModule(int driveId, int turnId, Rotation2d angleOffset) {
        driveMotor = new TalonFX(driveId, "*");
        turnMotor = new TalonFX(turnId, "*");

        driveMotor.getConfigurator().apply(SwerveModuleConfig.driveConfig);
        turnMotor.getConfigurator().apply(SwerveModuleConfig.turnConfig);

        this.angleOffset = angleOffset;

        ffModel = new SimpleMotorFeedforward(0, 0);

        driveMotor.setPosition(0);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnMotor.getPosition().getValueAsDouble()).minus(angleOffset);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble()) * Swerve.WHEEL_RADIUS, 
            getAngle()
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Units.rotationsToRadians(driveMotor.getVelocity().getValueAsDouble()) * Swerve.WHEEL_RADIUS,
            getAngle()
        );
    }

    public void setDriveVelocity(double velocity, double feedforward) {
        driveMotor.setControl(new VelocityTorqueCurrentFOC(Units.radiansToRotations(velocity)).withFeedForward(feedforward));
    }

    public void setTurnPosition(Rotation2d rotation) {
        turnMotor.setControl(new PositionTorqueCurrentFOC(rotation.getRotations()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState.optimize(getAngle());
        desiredState.cosineScale(getAngle());

        desiredState.speedMetersPerSecond /= Swerve.WHEEL_RADIUS;

        setDriveVelocity(desiredState.speedMetersPerSecond, ffModel.calculate(desiredState.speedMetersPerSecond));
        setTurnPosition(desiredState.angle.plus(angleOffset));
    }
}
