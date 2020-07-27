/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.constants;

import org.frcteam2910.common.control.PidConstants;

/**
 * Add your docs here.
 */
public class ChassisConstants {

    public static final double TRACKWIDTH = 19.5;
    public static final double WHEELBASE = 23.5;

    public static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(0.0);
    public static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(0.0);
    public static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(0.0);
    public static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(0.0);

    //These will probably need to be changed for the actual robot
    public static final PidConstants ANGLE_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001); //Angle constants, angle reduction, drive reduction and wheel diameter are defaults
    public static final double ANGLE_REDUCTION = 18.0 / 1.0;
    public static final double WHEEL_DIAMETER = 4.0;
    public static final double DRIVE_REDUCTION = 8.31 / 1.0;

    public static final double INITIAL_SPEED_SCALE = 0.25;
    public static final int NUM_SPEEDS = 4;
    public static final double SPEED_SCALE_INTERVAL = (1 - INITIAL_SPEED_SCALE) / (NUM_SPEEDS - 1);

}
