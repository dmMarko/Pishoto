/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.control;

import frc.LibPurple.swerve.Utils.Utils;

/**
 * Add your docs here.
 */
public class RotationPIDController extends PIDController{
    public RotationPIDController(PIDvalue constants){
        super(constants);
    }

    @Override
    public double calculate(double current, double dt) {
        setSetpoint(0);
        // current = (current + 2 * Math.PI) % (2 * Math.PI);
        // current = convert0To2Pi(current);
        return super.calculate(current, dt);
    }
    
    private double convert0To2Pi(double current){
        if (getSetpoint() > Math.PI && Utils.inRange(current, 0, Math.PI / 180)){
            current = 2 * Math.PI;
        }
        return current;
    }
}
