// package frc.LibPurple.swerve.controlTrajectory;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.LibPurple.control.Loop;
// import frc.LibPurple.math.RigidTransform2D;
// import frc.LibPurple.math.Rotation2D;
// import frc.LibPurple.math.Vector2D;
// import frc.LibPurple.swerve.Swerve.SwerveDrivetrain;
// import frc.LibPurple.swerve.Swerve.SwerveModule;
// import frc.LibPurple.swerve.trajectories.Segment;
// import frc.LibPurple.swerve.trajectories.Trajectory;
// import frc.LibPurple.utils.Utils;


// public abstract class TrajectoryController extends Loop
// {
// 	protected Vector2D pos = Vector2D.ZERO;
// 	protected SwerveDrivetrain drivetrain;
// 	protected double velocity;
// 	private double forwardSpeed, strafeSpeed, rotationSpeed;

// 	protected int direction;
    
// 	protected Trajectory trajectory;
	
//      // fixed you alon <3
//     // protected boolean taskWait = false;
	
// 	protected double[] setpoint;
// 	protected Segment setpointSegment;
// 	protected double startTime;
	

// 	protected double passedTime;
// 	protected double now;
// 	protected double dt;
// 	protected double lastTime;

// 	private Rotation2D adjustmentAngle = Rotation2D.ZERO;
	

// 	public TrajectoryController(SwerveDrivetrain drivetrain)
// 	{
//         super();
//         // this.modules = new SwerveModule3075[modules.length];
//         // for(int i = 0; i < this.modules.length; i++){
//         //     this.modules[i] = modules[i];
// 		// }
// 		this.drivetrain = drivetrain;
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
// 		pos = Vector2D.ZERO;
// 		setpointSegment = new Segment(Vector2D.ZERO, Rotation2D.ZERO, Rotation2D.ZERO, 0, 0);
//     }

// 	public void doCalculate()
// 	{
// 		now = Timer.getFPGATimestamp();
// 		dt = now - lastTime;
// 		passedTime = now - startTime;
// 		lastTime = now;
		
// 		double robotRotation = Math.toRadians(drivetrain.navX.getYaw());
        
//         Vector2D averageCenter = Vector2D.ZERO;
//         for(SwerveModule module : drivetrain.getSwerveModules()){
//             module.updateSensors();
//             module.updateKinematics(robotRotation);

//             Vector2D estimatedCenter = new RigidTransform2D(module.getCurrentPos(),
//                     Rotation2D.fromRadians(robotRotation))
//                     .transformBy(new RigidTransform2D(module.getModulePos().inverse(), Rotation2D.ZERO)).translation;

//             averageCenter = averageCenter.add(estimatedCenter);
//         }
//         averageCenter = averageCenter.scale(1.0 / drivetrain.getSwerveModules().length);

// 		pos = averageCenter;

// 		SmartDashboard.putNumber("lo x", pos.x);
// 		SmartDashboard.putNumber("lo y", pos.y);
		
// 		calculate(passedTime, dt);
// 	}

// 	// override me senpai
// 	public abstract void calculate(double passedTime, double dt);
	
// 	protected void setUnicycle(double vX, double vY, double vRotation){
// 		// forwardSpeed = vX;
// 		// strafeSpeed = vY;
// 		// rotationSpeed = vRotation;

// 		// Vector2D translation = new Vector2D(forwardSpeed, strafeSpeed);

// 		// // drivetrain.holonomicDrive(translation, rotationSpeed, true);
//         // Rotation2D unAdjustedAngle = Rotation2D.fromDegrees(-drivetrain.navX.getYaw());
//         // Rotation2D angle = unAdjustedAngle.rotateBy(adjustmentAngle);
//         // translation = translation.rotateBy(angle);

//         // for(int i=0; i < drivetrain.getSwerveModules().length; i++){
// 		// 	Vector2D vr = translation.scale(drivetrain.getSwerveModules()[i].getDistancePerPulse() / 10);
//         //     double a;
//         //     if(i == 0)
//         //     {
//         //         a = Math.PI / 4;
//         //     }
//         //     else if(i == 1)
//         //     {
//         //         a = Math.toRadians(135);
//         //     }
//         //     else if(i == 2)
//         //     {
//         //         a = Math.toRadians(315);
//         //     }
//         //     else
//         //     {
//         //         a = Math.toRadians(225);
//         //     }
//         //     double x = rotationSpeed * Math.cos(a);
//         //     double y = rotationSpeed * Math.sin(a);
            
// 		// 	Vector2D velocity = vr.add(new Vector2D(x, y));
			
//         //     drivetrain.getSwerveModules()[i].setTargetVelocity(velocity);
//         // }

// 		// drivetrain.updateKinematicsAuto(passedTime);
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
// 		passedTime = 0;
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

// 	public double getForwardSpeed()
// 	{
// 		return this.forwardSpeed;
// 	}

// 	public double getStrafeSpeed()
// 	{
// 		return this.strafeSpeed;
// 	}
	
// 	public double[] getSetpoint() 
// 	{
// 		return setpoint;
// 	}
	
// 	public double getPassedTime()
// 	{
// 		return this.passedTime;
// 	}
	
// 	public Vector2D getPos(){
// 		return this.pos;
// 	}

// 	public Segment getSetpointSegment(){
// 		return setpointSegment;
// 	}

// 	public void resetLocalization()
// 	{
// 		this.pos = Vector2D.ZERO;
// 	}
// 	public abstract double getSetpointVelocity();
// }
