package frc.LibPurple.swerve.trajectories;

public interface Trajectory 
{
    // public double[] calculate(double time);
    public Segment calculate(double time);

    // public double[] getNextSetpoint();
    public Segment getNextSetpoint();

    public Segment getLastSegment();

    public abstract double getTotalTime();
	
    public abstract int getDirection();
}