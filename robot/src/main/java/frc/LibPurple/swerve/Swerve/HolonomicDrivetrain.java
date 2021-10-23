/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.swerve.Swerve;

import frc.LibPurple.math.Vector2D;

/**
 * Add your docs here.
 */
public abstract class HolonomicDrivetrain extends Drivetrain{
    
    public final void holonomicDrive(Vector2D translation, double rotation) {
        holonomicDrive(translation, rotation, false);
    }

    public abstract void holonomicDrive(Vector2D translation,
     double rotation, boolean fieldOriented);
}
