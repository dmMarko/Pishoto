/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.control;

import frc.LibPurple.math.RandomMath;

/**
 * Add your docs here.
 */
public class PIDTalonController {
    private PIDvalue constants;

    private double setpoint;

    private boolean continuous = true;
    private double inputRange = Double.POSITIVE_INFINITY;
    private double minOutput = Double.NEGATIVE_INFINITY;
    private double maxOutput = Double.POSITIVE_INFINITY;

    private double lastError = Double.NaN;
    private double integralAccum = 0.0;
    private double integralRange = Double.POSITIVE_INFINITY;

    private boolean notFirst = true;
    private double IZone = Double.NaN;

    public PIDTalonController(PIDvalue constants) {
        this.constants = constants;
    }

    public double calculate(double current, double dt) {
        double error = setpoint - current;
        double absError = error;
        if(error < 0){
            absError = -absError;
        }

        if(notFirst){
            integralAccum = 0;
        }
        else if(!Double.isFinite(IZone) || absError < IZone){
            integralAccum += error;
        }
        else{
            integralAccum = 0;
        }

        double dError = 0.0;
        if(!notFirst){
            dError = (error - lastError) / dt;
        }

        lastError = error;
        notFirst = false;
        return RandomMath.clamp(constants.kP * error + constants.kI * integralAccum + constants.kD * dError,
                minOutput, maxOutput);
    }

    public void reset() {
        lastError = Double.NaN;
        integralAccum = 0.0;
        notFirst = true;
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

    public void disable(){
        calculate(0, 0);
    }
}
