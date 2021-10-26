package frc.LibPurple.swerve.trajectories;

import frc.LibPurple.math.Rotation2D;
import frc.LibPurple.math.Vector2D;

public class TimeStamp{

	public double time;

	public Vector2D translation;
    public Rotation2D heading;
    public Rotation2D rotation;
    public double velocity;

	public TimeStamp(double time, Vector2D translation, Rotation2D heading, Rotation2D rotation, double velocity)
	{
		this.time = time;
		this.translation = translation;
		this.heading = heading;
		this.rotation = rotation;
		this.velocity = velocity;
	}

}