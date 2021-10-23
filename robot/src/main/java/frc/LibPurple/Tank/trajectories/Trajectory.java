package frc.LibPurple.Tank.trajectories;

public interface Trajectory 
{
    public double[] calculate(double time);

    public double[] getNextSetpoint();

    public abstract double getTotalTime();
	
    public abstract int getDirection();
}