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
public class HolonomicDriveSignal {
    private final Vector2D translation;
    private final double rotation;
    private final boolean fieldOriented;

    public HolonomicDriveSignal(Vector2D translation, double rotation, boolean fieldOriented) {
        this.translation = translation;
        this.rotation = rotation;
        this.fieldOriented = fieldOriented;
    }

    public Vector2D getTranslation() {
        return translation;
    }

    public double getRotation() {
        return rotation;
    }

    public boolean isFieldOriented() {
        return fieldOriented;
    }
}
