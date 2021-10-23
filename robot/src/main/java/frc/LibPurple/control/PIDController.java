/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.control;

import frc.LibPurple.math.RandomMath;
import frc.LibPurple.swerve.Utils.Utils;

/**
 * Add your docs here.
 */
public class PIDController {
    protected PIDvalue constants;

    protected double setpoint;

    protected boolean continuous = false;
    protected double inputRange = Double.POSITIVE_INFINITY;
    protected double minOutput = Double.NEGATIVE_INFINITY;
    protected double maxOutput = Double.POSITIVE_INFINITY;

    protected double lastError = Double.NaN;
    protected double integralAccum = 0.0;
    protected double integralRange = Double.POSITIVE_INFINITY;

    protected double derivative = 0;

    protected double current;

    public PIDController(PIDvalue constants) {
        this.constants = constants;
    }

    public double calculate(double current, double dt) {
        this.current = current;
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
        if (Math.abs(error) < integralRange / 2.0) {
            integral = integralAccum + error * dt;
        }
        integralAccum = integral;

        double derivative = 0.0;
        if (Double.isFinite(lastError)) {
            derivative = (error - lastError) / dt;
        }
        this.derivative = derivative;
        lastError = error;
        return RandomMath.clamp(constants.kP * error + constants.kI * integral + constants.kD * derivative,
                minOutput, maxOutput);
    }

    public double getD(){
        return derivative;
    }

    public void reset() {
        lastError = Double.NaN;
        integralAccum = 0.0;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getLastError(){
        return lastError;
    }

    public double getIntegral(){
        return integralAccum;
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

    public boolean onTarget(double target, double tolerans){
        return Utils.inRange(target, setpoint, tolerans);
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

    // public void disable(){
    //     calculate(0, 0);
    // }
}
