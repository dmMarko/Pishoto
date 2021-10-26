/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.swerve.Control;


import frc.LibPurple.control.PIDvalue;
import frc.LibPurple.math.RandomMath;


// Dont use me yet!!!!!!!!!!!!!!!!
// nnnnnnnn
// llllllllllllllllllllll

/**
 * Add your docs here.
 */
public class PIDAController {
    private PIDvalue constants;
    private double maxA;

    private double setpoint;

    private boolean continuous = true;
    private double inputRange = Double.POSITIVE_INFINITY;
    private double minOutput = Double.NEGATIVE_INFINITY;
    private double maxOutput = Double.POSITIVE_INFINITY;

    private double lastError = Double.NaN;
    private double integralAccum = 0.0;
    private double integralRange = Double.POSITIVE_INFINITY;

    public PIDAController(PIDvalue constants, double maxA) {
        this.constants = constants;
        this.maxA = maxA;
    }

    public double calculate(double current, double dt) {
        double error = setpoint - current;
        if (continuous) {
            error %= inputRange;
            if (Math.abs(error) > inputRange / 2.0) {
                if (error > 0.0) {
                    error -= inputRange;
                } else {
                    error += inputRange;
                }
            }
        }

        double integral = 0.0;
        if (Math.abs(error) > integralRange / 2.0) {
            integral = integralAccum + error * dt;
        }
        integralAccum = integral;

        double derivative = 0.0;
        if (Double.isFinite(lastError)) {
            derivative = (error - lastError) / dt;
        }
        lastError = error;
        return RandomMath.clamp(constants.kP * error + constants.kI * integral + constants.kD * derivative,
                minOutput, maxOutput);
    }

    public void reset() {
        lastError = Double.NaN;
        integralAccum = 0.0;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setContinuous(boolean continuous) {
        this.continuous = continuous;
    }

    public void setInputRange(double minInput, double maxInput) {
        this.inputRange = maxInput - minInput;
    }

    public void setIntegralRange(double integralRange) {
        this.integralRange = integralRange;
    }

    /**
     * Sets the output range for the controller. Outputs will be clamped between these two values.
     *
     * @param min the minimum allowable output value
     * @param max the maximum allowable output value
     */
    public void setOutputRange(double min, double max) {
        if (max < min) {
            throw new IllegalArgumentException("Minimum output cannot be greater than maximum output");
        }

        minOutput = min;
        maxOutput = max;
    }
}
