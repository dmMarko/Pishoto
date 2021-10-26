/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.swerve.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.LibPurple.math.Rotation2D;
import frc.LibPurple.math.Vector2D;
import frc.LibPurple.swerve.Utils.Utils;


/**
 * Add your docs here.
 */
public abstract class SwerveModule {
    protected final Vector2D modulePos;
    private double currentAngle = 0.0;
    private double currentDistance = 0.0;

    protected double targetSpeed = 0.0;
    protected double targetAngle = 0.0;
    protected boolean isFillp = false;

    private Vector2D currentPos = Vector2D.ZERO;
    private double previousDistance;
    private String moduleName;

    public SwerveModule(Vector2D modulePos){
        this.modulePos = modulePos;
    }

    public void setModuleName(String moduleName){
        this.moduleName = moduleName;
    }

    public String getModuleName(){
        return moduleName;
    }

    public double getTargetAngle(){
        return targetAngle;
    }

    public abstract double getAngle();

    public abstract double getDistance();

    public abstract double getError();

    public abstract void setTargetAngle(double angle);

    public abstract void setAngleOutput(double angle);

    public abstract void setDriveOutput(double output);

    public abstract void setVelocity(double velocity);

    public abstract double getSetDrive();

    public abstract double getSetAngle();

    public abstract double getVelocity();

    public abstract void resetPID();

    public final Vector2D getModulePos(){
        return modulePos;
    }

    public final double getCurrentAngle(){
        return currentAngle;
    }

    public final double getCurrentDistance(){
        return currentDistance;
    }

    public double getCurrentVelocity(){
        return 0;
    }

    public double getDriveCurrent(){
        return 0;
    }

    public abstract double getDistancePerPulse();

    public Vector2D getTargetVelocity(){
        return Vector2D.fromAngle(Rotation2D.fromRadians(targetAngle)).scale(targetSpeed);
    }

    public final void setTargetVelocity(Vector2D velocity){
        targetSpeed = velocity.length;
        targetAngle = velocity.getAngle().toRadians();
        SmartDashboard.putNumber("taret", targetAngle);
    }

    public final void setTargetVelocity(double speed, double angle){
        // if(speed < 0.0){
            // speed *= -1.0;
            // angle += Math.PI;
        // }
        // angle %= 2.0 * Math.PI;

        // if(angle < 0.0){
        //     angle += 2.0 * Math.PI;
        // }

        targetSpeed = speed;
        targetAngle = angle;
    }

    public final Vector2D getCurrentPos(){
        return currentPos;
    }

    public void resetKinematics(){
        resetKinematics(Vector2D.ZERO);
        previousDistance = 0;
    }

    public void resetKinematics(Vector2D pos){
        currentPos = pos;
    }

    public void updateSensors(){
        currentAngle = Math.toRadians((int) Math.toDegrees(getAngle()));
        currentDistance = getDistance();
    }

    public void updateKinematics(double robotRotation){
        double currentDistance = getCurrentDistance();
        double deltaDistance = currentDistance - previousDistance;
        double currentAngle = getCurrentAngle() + robotRotation;
        Vector2D deltaPos = new Vector2D(Math.cos(currentAngle) * deltaDistance, Math.sin(currentAngle) * deltaDistance);
        
        currentPos = currentPos.add(deltaPos);
        previousDistance = currentDistance;
    }

    public void updateState(double dt){
        final double currentAngle = getCurrentAngle();

        double delta = targetAngle - currentAngle;
        // double absError = targetAngle - getAngle();
        // double error = 0;
        // if(delta > -Math.PI){
        //     error = delta;
        // }
        // else{
        //     error = delta + 2 * Math.PI;
        // }
        // targetAngle = error + currentAngle;
        
        // if(delta > Math.PI / 2 || delta < -Math.PI / 2){
            // targetAngle += Math.PI;
            // targetSpeed *= -1;
        // }
        if(delta >= Math.PI){
            targetAngle -= 2.0 * Math.PI;
        } else if(delta < -Math.PI){
            targetAngle += 2.0 * Math.PI;
        }

        delta = targetAngle - currentAngle;
        if(delta > Math.PI / 2.0 || delta < -Math.PI / 2.0){
            targetAngle += Math.PI;
            targetSpeed *= -1.0;
        }
        
        // Utils.print("1 " + targetAngle);
        targetAngle %= 2.0 * Math.PI;
        // Utils.print("2 " + targetAngle);
        if (targetAngle < 0.0) {
            targetAngle += 2.0 * Math.PI;
        }

        // Utils.print("" + targetSpeed);
        // targetAngle = 0;

        setTargetAngle(targetAngle);
        // Utils.print("" + targetSpeed * 6000.0 * 2048.0 / 600.0);
        // setVelocity(Math.signum(targetSpeed) * 2000.0 * 2048.0 / 600.0);
        // setVelocity(targetSpeed * 6000.0 * 2048.0 / 600.0);
        // setVelocity(targetSpeed * 2000.0 * 2048.0 / 600.0);
        setDriveOutput(targetSpeed);
    }

    public void updateStateAuto(double dt){

        double delta = targetAngle - currentAngle;
        if(delta >= Math.PI){
            targetAngle -= 2.0 * Math.PI;
        } else if(delta < -Math.PI){
            targetAngle += 2.0 * Math.PI;
        }

        // delta = targetAngle - currentAngle;
        // if(delta > Math.PI / 2.0 || delta < -Math.PI / 2.0){
        //     targetAngle += Math.PI;
        //     targetSpeed *= -1.0;
        // }
        
        // Utils.print("1 " + targetAngle);
        // targetAngle %= 2.0 * Math.PI;
        // // Utils.print("2 " + targetAngle);
        // if (targetAngle < 0.0) {
        //     targetAngle += 2.0 * Math.PI;
        // }

        delta = targetAngle - currentAngle;
        if(delta > Math.PI / 2.0 || delta < -Math.PI / 2.0){
            targetAngle += Math.PI;
            targetSpeed *= -1.0;
        }
        
        // Utils.print("1 " + targetAngle);
        targetAngle %= 2.0 * Math.PI;
        // Utils.print("2 " + targetAngle);
        if (targetAngle < 0.0) {
            targetAngle += 2.0 * Math.PI;
        }
        

        // Utils.print("" + targetSpeed);
        // targetAngle = 0;

        SmartDashboard.putNumber("angleTar", targetAngle);
        SmartDashboard.putNumber("speedTar", targetSpeed);

        setTargetAngle(targetAngle);
        // setDriveOutput(targetSpeed);
        setVelocity(targetSpeed);
    }
}
