/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.swerve.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.LibPurple.math.Vector2D;

/**
 * Add your docs here.
 */
public abstract class Drivetrain extends SubsystemBase{
    public abstract double getMaximumVelocity();
    public abstract double getMaximumAcceleration();

	public abstract void updateKinematics(double timestamp);

	public abstract void updateKinematicsAuto(double timestamp);

	public abstract Vector2D getKinematicPosition();

	public abstract Vector2D getKinematicVelocity();

	public void outputToSmartDashboard() {
		SmartDashboard.putString("Drivetrain position", getKinematicPosition().toString());

		// SmartDashboard.putNumber("Drivetrain angle", Robot.navX.getYaw());
	}
	
	public void zeroSensors() {}
}
