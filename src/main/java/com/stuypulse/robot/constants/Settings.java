/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Swerve {
        double WHEEL_RADIUS = 0;

        double LENGTH = 0;
        double WIDTH = 0;

        SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
            new Translation2d[] {
                new Translation2d(WIDTH, LENGTH),
                new Translation2d(-WIDTH, LENGTH),
                new Translation2d(WIDTH,-LENGTH),
                new Translation2d(-WIDTH, -LENGTH)
            }
        );
    }
}
