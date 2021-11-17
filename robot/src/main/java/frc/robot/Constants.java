/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.LibPurple.control.PIDvalue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Game
    public static final double TAKE_DISC_HIGHT = 10000;


    // intake
    public static final double COLLECT_DISC_FROM_FLOOR_SPIN_POWER = .5;
    public static final double COLLECT_BALL_TOP_SPIN_POWER = -0;
    public static final double COLLECT_BALL_BOTTOM_SPIN_POWER = 0;

    // drive
    // The location are relative to the arm being far away from you.
    public static final PIDvalue LEFT_SIDE_PID = new PIDvalue(1, 0, 0);
    public static final PIDvalue RIGHT_SIDE_PID = new PIDvalue(1, 0, 0);

    // arm
    public static final PIDvalue ARM_PID = new PIDvalue(.9, .0009, 0.4);
    public static final double ARM_MIN_POS = 0;
    public static final double ARM_MAX_POS = 3639;
    public static final double ARM_MAX_POWER_FORWARD = .4;
    public static final double ARM_MAX_POWER_REVERSE = -0.4;
    // arm positions
    public static final double ARM_FLOOR_ANGLE = 0;
    public static final double ARM_DEFAULT_ANGLE = 70 ;
    
    // elevator
    public static final PIDvalue ELEVATOR_PID = new PIDvalue(.2, .00003, 0.1);
    public static final double ELEVATOR_MAX_POS = -47500; //- 54150;    
    public static final double ELEVATOR_MAX_POWER_FORWARD = .7;
    public static final double ELEVATOR_MAX_POWER_REVERSE = -0.29;
}
