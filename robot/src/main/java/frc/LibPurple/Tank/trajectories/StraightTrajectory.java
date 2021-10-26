package frc.LibPurple.Tank.trajectories;


public class StraightTrajectory implements Trajectory
{
    protected double T;
	protected double distance;
    protected int direction;
    protected double[] setpoint;
    protected double maxV;
    protected double maxA;
    protected double t1;
    protected double t2;
    protected double triangleArea;
    protected double rectangleArea;
    private double time;

    public StraightTrajectory(double distance, double maxV, double maxA)
    {
        this.direction = (int)Math.signum(distance);
        this.distance = distance;
        this.setpoint = new double[3];
		this.maxA = maxA * direction;
        this.maxV = maxV * direction;
        this.T = Math.sqrt(4 * this.distance / maxA);
        double triangleMaxV = 0.5 * this.T * maxA;
        if (Math.abs(triangleMaxV) > Math.abs(this.maxV))
        {
            t1 = maxV / maxA;
            this.triangleArea = t1 * maxV / 2;    
            this.rectangleArea = distance - this.triangleArea * 2;
            t2 = rectangleArea / maxV;
            this.T = t1 + t1 + t2;
        }
        else
        {
            t1 = 0.5 * this.t1;
            t2 = 0;           
            this.triangleArea = t1 * maxV / 2;
            this.rectangleArea = distance - this.triangleArea * 2;
         }
        
    }

    @Override
    public double[] calculate(double time)
    {		
        this.time = time;
		if(time <= t1)
		{
			double v = maxA * time;
			double p = (time * v) / 2;
            setpoint[0] = p;
		}
		else if(time < T - t1)
		{
			double p = ((time - t1) * maxV) + this.triangleArea;
            setpoint[0] = p;
		}
		else if(time <= T)
		{
			double v = -maxA * (time - T);
			double p = (((time - t2 - t1) * (maxV + v)) / 2) + this.triangleArea + this.rectangleArea;
            setpoint[0]= p;
        }
        setpoint[1] = 0;
        setpoint[2] = 0;
		return setpoint;
    }

    @Override
    public double getTotalTime() {
        return this.T;
    }

    @Override
    public int getDirection() {
        return this.direction;
    }

    public double getMaxVelocity() {
        return this.maxV;
    }

    public double getMaxAcceleration() {
        return this.maxA;
    }

    @Override
    public double[] getNextSetpoint() {
        return calculate(time + 0.01);
    }

}