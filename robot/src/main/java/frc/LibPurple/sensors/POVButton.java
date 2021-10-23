package frc.LibPurple.sensors;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;

public class POVButton extends Button
{
	private double angle;
	private Joystick stick;
	
	public POVButton(Joystick stick, double angle)
	{
		this.angle = angle;
		this.stick = stick;
	}
	
	@Override
	public boolean get()
	{
		return stick.getPOV() == angle;
	}

}
