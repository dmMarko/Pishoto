package frc.LibPurple.Tank.trajectories;


public class StraightTrajectorySMP implements Trajectory
{
    private double T;
	private double K;
	private double distance;
    private int direction;
    private double maxA;
    private double time;


    public StraightTrajectorySMP(double distance, double maxA)
    {
        this.direction = (int) Math.signum(distance);
        this.distance = distance;
        this.maxA = maxA;
		
		T = Math.sqrt((Math.abs(this.distance * Math.PI * 2)) / maxA);
		K = (2 * Math.PI) / T;
    }

    @Override
    public double[] calculate(double time)
    {
        this.time = time;
        if (time > T)
            time = T;
        double[] setpoint = new double[3];
		setpoint[0] = (-(maxA / (K * K)) * Math.sin(K * time) + maxA/K * time) * this.direction;
		// setpoint.velocity = (-(maxA / K) * Math.cos(K * time) + maxA/K) * this.direction;
		// setpoint.acceleration = (maxA * Math.sin(K * time)) * this.direction;
        setpoint[1] = 0;
        setpoint[2] = 0;
		return setpoint;

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
    public double[] getNextSetpoint() 
    {
        return calculate(this.time + 0.01);
    }
}