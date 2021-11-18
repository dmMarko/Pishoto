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
    // drive
    // The location are relative to the arm being far away from you.
    public static final PIDvalue leftSidePID = new PIDvalue(1, 0, 0);
    public static final PIDvalue rightSidePID = new PIDvalue(1, 0, 0);
    public static final double oneRoundTicks = 2463;
    public static final double COLLECT_DISK_FROM_FLOOR_SPIN_POWER = 0;
}
