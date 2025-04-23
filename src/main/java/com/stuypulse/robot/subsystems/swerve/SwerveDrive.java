package com.stuypulse.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.stuypulse.robot.constants.Settings.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private final static SwerveDrive instance;
    
    static {
        instance = new SwerveDrive(
            new SwerveModule(0,0,new Rotation2d()),
            new SwerveModule(0,0,new Rotation2d()),
            new SwerveModule(0,0,new Rotation2d()),
            new SwerveModule(0,0,new Rotation2d())
        );
    }

    public SwerveDrive getInstance() {
        return instance;
    }

    private final SwerveModule[] modules;

    private final Pigeon2 gyro;

    private final SwerveDriveOdometry odometry;

    private SwerveModulePosition[] modulePositions;

    public SwerveDrive(SwerveModule... modules) {
        this.modules = modules;

        gyro = new Pigeon2(0, "*");

        odometry = new SwerveDriveOdometry(
            Swerve.driveKinematics,
            gyro.getRotation2d(),
            modulePositions
        );
    }

    public void drive(Translation2d velocity, double rotation, boolean fieldRelative) {
        SwerveModuleState[] desiredStates = Swerve.driveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(velocity.getX(), velocity.getY(), rotation, gyro.getRotation2d())
                : new ChassisSpeeds(velocity.getX(), velocity.getY(), rotation)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 0);
        
        setModuleStates(desiredStates);
    }
    
    public void setX() {
        SwerveModuleState[] desiredStates = new SwerveModuleState[] {
            new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45))),
            new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45))),
            new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45))),
            new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45))),
        };

        setModuleStates(desiredStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        for(int i=0; i<4; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    public void resetOdometry(Pose2d targetPose) {
        odometry.resetPosition(
            gyro.getRotation2d(), 
            modulePositions,
            targetPose
        );
    }

    @Override
    public void periodic() {
        modulePositions = new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[0].getPosition()
        };

        odometry.update(
            gyro.getRotation2d(),
            modulePositions
        );

        SmartDashboard.putNumber("FL Angle (Rotations)", modules[0].getAngle().getRotations());
        SmartDashboard.putNumber("FR Angle (Rotations)", modules[0].getAngle().getRotations());
        SmartDashboard.putNumber("BL Angle (Rotations)", modules[0].getAngle().getRotations());
        SmartDashboard.putNumber("BR Angle (Rotations)", modules[0].getAngle().getRotations());
    }
}
