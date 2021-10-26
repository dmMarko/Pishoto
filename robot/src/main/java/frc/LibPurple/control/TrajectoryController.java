// package frc.LibPurple.control;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import edu.wpi.first.wpilibj.Timer;
// import frc.LibPurple.trajectories.Trajectory;
// import frc.LibPurple.utils.Utils;
// import frc.robot.Constants;
// import frc.robot.Robot;

// public abstract class TrajectoryController extends Loop
// {
// 	protected double x = 0, y = 0;
// 	protected WPI_TalonSRX motorRight;
// 	protected WPI_TalonSRX motorLeft;
// 	protected double velocity;
// 	private double rightSpeed, leftSpeed;

// 	protected int direction;
    
// 	protected Trajectory trajectory;
	
//      // fixed you alon <3
//     // protected boolean taskWait = false;
	
// 	protected double[] setpoint;
// 	protected double startTime;
	

// 	protected double passedTime;
// 	protected double now;
// 	protected double dt;
// 	protected double lastTime;
	

// 	public TrajectoryController(WPI_TalonSRX motorRight, WPI_TalonSRX motorLeft)
// 	{
// 		super();
// 		this.motorRight = motorRight;
// 		this.motorLeft = motorLeft;
// 		// double[] firstSetPoint = {0.0, 0.0, 90.0};
// 		// this.setpoint = firstSetPoint;
// 		passedTime = 0;
// 	}
	
// 	@Override
// 	public void run() 
// 	{	
// 		if (!enabled) 
//         {
//             disable();
// 			return;
// 		}
// 		doCalculate();	
// 	}

// 	public void setTrajectory(Trajectory trajectory)
//     {
//     	Utils.print("total time= " + trajectory.getTotalTime());
//     	this.trajectory = trajectory;
//     	this.setpoint = new double[3];
// 		this.lastTime = 0;
// 		this.now = 0;
//     }

// 	public void doCalculate()
// 	{
// 		now = Timer.getFPGATimestamp();
// 		// dt = now - startTime - passedTime;
// 		dt = now - lastTime;
// 		passedTime = now - startTime;
// 		lastTime = now;
// 		double angleRad = Math.toRadians(Robot.driveSystem.navx.getYaw());
//         this.x += ((Robot.driveSystem.getLeftEncoder().getRate() + Robot.driveSystem.getRightEncoder().getRate()) / 2.0) * Math.cos(angleRad) * dt;
// 		this.y += ((Robot.driveSystem.getLeftEncoder().getRate() + Robot.driveSystem.getRightEncoder().getRate()) / 2.0) * Math.sin(angleRad) * dt;
// 		calculate(passedTime, dt);
// 	}

// 	// override me senpai
//     public abstract void calculate(double passedTime, double dt);
	
// 	protected void setUnicycle(double v, double w)
// 	{
// 		// w *= this.direction;
// 		leftSpeed = ((2.0 * v) + (w * Constants.robotWidth)) / 2.0;
// 		rightSpeed = ((2.0 * v) - (w * Constants.robotWidth)) / 2.0;
		
// 		this.motorLeft.set(ControlMode.Velocity, leftSpeed * Constants.leftDistancePerPulse / 10); // TODO why the / 10 ?
// 		this.motorRight.set(ControlMode.Velocity, rightSpeed * Constants.rightDistancePerPulse / 10);
// 	}

// 	public void setDirection(boolean reverse)
// 	{
// 		this.direction = reverse ? -1 : 1;
// 	}

// 	public void enable()
// 	{
// 		Utils.print("drive enabled");
// 		super.enable();
// 		startTime = Timer.getFPGATimestamp();
// 		lastTime = startTime;
// 	}

// 	public boolean isTimeUp()
// 	{
// 		return passedTime > trajectory.getTotalTime();
// 	}
	
// 	public boolean isEnabled()
// 	{
// 		return enabled;
// 	}
	
// 	public Trajectory getTrajectory() 
// 	{
// 		return trajectory;
// 	}	

// 	public double getRightSpeed()
// 	{
// 		return this.rightSpeed;
// 	}

// 	public double getLeftSpeed()
// 	{
// 		return this.leftSpeed;
// 	}
	
// 	public double[] getSetpoint() 
// 	{
// 		return setpoint;
// 	}
	
// 	public double getPassedTime()
// 	{
// 		return this.passedTime;
// 	}
	
// 	public double getX()
// 	{
// 		return x;
// 	}

// 	public double getY()
// 	{
// 		return y;
// 	}
// 	public void resetLocalization()
// 	{
// 		Robot.driveSystem.navx.resetYaw();
// 		this.x = 0;
// 		this.y = 0;
// 	}
// 	public abstract double getSetpointVelocity();
// }
