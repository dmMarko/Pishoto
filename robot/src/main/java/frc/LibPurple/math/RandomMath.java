/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.math;

/**
 * Add your docs here.
 */
public class RandomMath {
    public static final double EPSILON = 1e-9;

    public static boolean epsilonEquals(double a, double b) {
		return epsilonEquals(a, b, EPSILON);
    }
    
    public static boolean epsilonEquals(double a, double b, double epsilon) {
		return Math.abs(a - b) < epsilon;
    }
    
    public static double clamp(double value, double min, double max) {
		if (min > max) {
			throw new IllegalArgumentException("min must not be greater than max");
		}

		return Math.max(min, Math.min(value, max));
	}
}
