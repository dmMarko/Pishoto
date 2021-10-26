package frc.LibPurple.swerve.trajectories;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.LibPurple.math.Rotation2D;
import frc.LibPurple.math.Vector2D;

public class StraightTrajectorySMP implements Trajectory
{
    private double T;
    private double K;
    private double distance;
    private int direction;
    private double maxA;
    private double time;

    private double angle;

    private Vector2D start;
    private Vector2D end;
    private Vector2D delta;

    private double rotation;


    public StraightTrajectorySMP(double distance, double angle, double rotation, double maxA)
    {
        this.direction = (int) Math.signum(distance);
        this.distance = distance;
        this.maxA = maxA;

        this.angle = angle;
		
		T = Math.sqrt((Math.abs(this.distance * Math.PI * 2)) / maxA);
        K = (2 * Math.PI) / T;

        start = new Vector2D(0, 0);
        end = new Vector2D(this.distance * Math.cos(Math.toRadians(this.angle)), this.distance * Math.sin(Math.toRadians(this.angle)));
        delta = end.subtract(start);

        this.rotation = rotation;
        
    }

    @Override
    public Segment calculate(double time)
    {
        this.time = time;
        if (time > T)
            time = T;

        double pos = (-(maxA / (K * K)) * Math.sin(K * time) + maxA / K * time) * this.direction;
        double velocity = (-(maxA / K) * Math.cos(K * time) + maxA / K) * this.direction;
        double acceleration = (maxA * Math.sin(K * time)) * this.direction;

        Vector2D translation = new Vector2D(pos * Math.cos(Math.toRadians(angle)), pos * Math.sin(Math.toRadians(angle)));

        Rotation2D heading = delta.getAngle();
        Rotation2D rotation = Rotation2D.fromDegrees(this.rotation * time / T);
        SmartDashboard.putNumber("forward", pos * Math.cos(Math.toRadians(angle)));
        SmartDashboard.putNumber("time control", time);

        return new Segment(translation, heading, rotation, velocity, acceleration);
    }

    @Override
    public double getTotalTime() 
    {
        return this.T;
    }

    @Override
    public int getDirection() 
    {
        return this.direction;
    }

    public double getMaxAcceleration() 
    {
        return this.maxA;
    }

    @Override
    public Segment getNextSetpoint() 
    {
        return calculate(this.time + 0.01);
    }

    @Override
	public Segment getLastSegment(){
		return calculate(T);
	}
}