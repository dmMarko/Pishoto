// package frc.LibPurple.control;

// import edu.wpi.first.wpilibj.Timer;
// import frc.LibPurple.sensors.Encoder3075;
// import frc.LibPurple.sensors.NavX;



// /**
//  * 
//  * @author 3075programming
//  * Our implementation of Motion Profiled control 
//  */
// public class Localizataion extends Loop
// {
    
// 	protected double startTime;
// 	protected double passedTime;
// 	protected double now;
// 	protected double dt;
//     protected double lastTime;
// 	protected double x = 0;
// 	protected double y = 0;
// 	protected double angle = 0;
// 	protected Encoder3075 rightEncoder, leftEncoder;
// 	protected NavX navX;

	
// 	public Localizataion(Encoder3075 rightEncoder, Encoder3075 leftEncoder, NavX navX) 
// 	{
// 		super();
// 		// this.period = 0.005;
// 		this.rightEncoder = rightEncoder;
// 		this.leftEncoder = leftEncoder;
// 		this.navX = navX;
// 	}
	
// 	@Override
// 	public void run() 
// 	{
// 		if (!enabled) 
//         {
//             disable();
// 			return;
// 		}
// 		calculate();	
// 	}

// 	public void calculate()
//     {
//     	now = Timer.getFPGATimestamp();
//         passedTime = now - startTime;
//     	double dt = passedTime - lastTime;
	
// 		this.angle = navX.getYaw();
//         this.x += ((this.leftEncoder.getRate() + this.rightEncoder.getRate()) / 2.0) * Math.cos(this.getAngleRad()) * dt;
// 		this.y += ((this.leftEncoder.getRate() + this.rightEncoder.getRate()) / 2.0) * Math.sin(this.getAngleRad()) * dt;
        
//         lastTime = passedTime;
//     }
	
// 	public void enable()
// 	{
// 		super.enable();
// 		startTime = Timer.getFPGATimestamp();
// 	}
	
// 	public double getPassedTime()
// 	{
// 		return this.passedTime;
//     }

//     public double getAngle()
//     {
//         return this.angle;
// 	}
	
// 	public double getAngleRad()
// 	{
// 		return Math.toRadians(this.angle);
// 	}

//     public double getX()
//     {
//         return this.x;
//     }

//     public double getY()
//     {
//         return this.y;
//     }

//     public void reset()
//     {
// 		navX.resetYaw();
// 		this.angle = 0;
//         this.x = 0;
//         this.y = 0;
//         this.now = 0;
//         this.passedTime = 0;
//         this.lastTime = 0;
//         this.startTime = 0;
// 	}
// }