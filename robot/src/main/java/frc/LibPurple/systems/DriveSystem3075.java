// package frc.LibPurple.systems;


// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import frc.robot.Constants;
// import frc.LibPurple.control.PIDvalue;
// import frc.LibPurple.math.Matrix;
// import frc.LibPurple.sensors.ConsoleJoystick;
// import frc.LibPurple.sensors.Encoder3075;
// import frc.LibPurple.sensors.EncoderTalon3075;
// import frc.LibPurple.sensors.NavX;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.Sendable;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// /***
//  * Initialization example: super.setPIDValues(...); super.setMPValues(...);
//  * super.setTurnMPValues(...); super.setVelocityTolerance(...);
//  * super.setDistanceTolerance(...); super.distancePerAngle = ... super.rightMaxV
//  * = ... super.leftMaxV = ... super.leftMaxA = ... super.rightMaxA = ...
//  * 
//  * @author 3075
//  *
//  */
// public abstract class DriveSystem3075 extends SubsystemBase implements Sendable
// {
// 	public static enum ControllerType 
// 	{
// 		FeedForward, WangLi, PurePursuit;
// 	}

// 	protected WPI_TalonSRX rightMotor;
// 	protected WPI_TalonSRX leftMotor;
// 	private Encoder3075 rightEncoder;
// 	private Encoder3075 leftEncoder;
// 	public NavX navx;
// 	protected PIDvalue leftVelocityPIDValue;
// 	protected PIDvalue rightVelocityPIDValue;
// 	protected PIDvalue leftTurnVelocityPIDValue;
// 	protected PIDvalue rightTurnVelocityPIDValue;
// 	protected TrajectoryController controller;

// 	public Encoder3075 getLeftEncoder() {
// 		return this.leftEncoder;
// 	}

// 	public Encoder3075 getRightEncoder() {
// 		return this.rightEncoder;
// 	}

// 	@Override
// 	protected void initDefaultCommand() {
// 		// TODO Auto-generated method stub
// 	}

// 	/**
// 	 * if the drive system is in velocity state then sets the system's speed if the
// 	 * drive system is in voltage state then gives the values straight to the
// 	 * engines
// 	 * 
// 	 * @param rightValue - the right engine speed/value
// 	 * @param leftValue  - the left engine speed/value
// 	 */
// 	public void set(double rightValue, double leftValue) {
// 		{
// 			rightMotor.set(rightValue);
// 			leftMotor.set(leftValue);
// 		}
// 	}

// 	public void setController(ControllerType controller)
// 	{
// 		if(controller == ControllerType.FeedForward)
// 		{
// 			if(!(this.controller instanceof FFController))
// 				this.controller = new FFController(this.rightMotor, this.leftMotor);
// 		}
// 		if(controller == ControllerType.WangLi)
// 		{
// 			if(!(this.controller instanceof WangLiController))
// 				this.controller = new WangLiController(this.rightMotor, this.leftMotor);
// 		}

// 		if(controller == ControllerType.PurePursuit)
// 		{
// 			if(!(this.controller instanceof PurePursuitController))
// 				this.controller = new PurePursuitController(this.rightMotor, this.leftMotor);
// 		}
// 	}

// 	/**
// 	 * resets the encoders
// 	 */
// 	public void reset() {
// 		leftEncoder.reset();
// 		rightEncoder.reset();
// 	}

// 	public Command arcadeDrive(Joystick joystick) {
// 		return new ArcadeDriveCommand(this, joystick);
// 	}

// 	public Command xboxArcadeDrive(ConsoleJoystick stick) {
// 		return new XBoxArcadeCommand(this, stick);
// 	}

// 	public Command tankDrive(Joystick rightJoystick, Joystick leftJoystick) {
// 		return new TankDriveCommand(this, rightJoystick, leftJoystick);
// 	}

// 	public Command driveStraight(double distance, ControllerType controllerType, boolean endless, boolean debug)
// 	{
// 		return new DriveStraight(this, distance, controllerType, endless, debug);
// 	}

// 	public Command driveStraight(double distance, double maxA, double maxV, ControllerType controllerType, boolean endless, boolean debug)
// 	{
// 		return new DriveStraight(this, distance, maxA, maxV, controllerType, endless, debug);
// 	}

// 	public Command driveArc(double radius, double ang, boolean clockWise, ControllerType controller, boolean debug)
// 	{
// 		return new DriveArc(this, radius, ang, clockWise, controller, debug);
// 	}
	
// 	public Command drive2DCommand(String name, boolean reversed, ControllerType controllerType, boolean debug) {
// 		return new Drive2DCommand(this, "/Paths/" + name + ".csv", reversed, controllerType, debug);
// 	}
	
// 	/**
// 	 *0 sets the PID values
// 	 * 
// 	 * @param leftPIDValue  - the values for the left side
// 	 * @param rightPIDValue - the values for the right side
// 	 */
// 	public void setPIDValues(PIDvalue leftPIDValue, PIDvalue rightPIDValue) 
// 	{
// 		this.leftMotor.config_kP(0, leftPIDValue.kP);
// 		this.leftMotor.config_kI(0, leftPIDValue.kI);
// 		this.leftMotor.config_kD(0, leftPIDValue.kD);
// 		this.leftMotor.config_kF(0, leftPIDValue.kF);
// 		this.rightMotor.config_kP(0, rightPIDValue.kP);
// 		this.rightMotor.config_kI(0, rightPIDValue.kI);
// 		this.rightMotor.config_kD(0, rightPIDValue.kD);
// 		this.rightMotor.config_kF(0, rightPIDValue.kF);
// 	}

// 	public double getAngle() {
// 		return navx.getYaw();
// 	}

// 	public double getAngleRad() {
// 		return Math.toRadians(this.getAngle());
// 	}

// 	public double getX() {
// 		return controller.getX();
// 	}

// 	public double getY() {
// 		return controller.getY();
// 	}

// 	// public SimpleMatrix getStateMatrix()
// 	public Matrix getStateMatrix()

// 	{
// 		// return new SimpleMatrix(3, 1, true, new double[]{this.getX(), this.getY(), this.getAngleRad()});
// 		return new Matrix(3, 1, new double[]{this.getX(), this.getY(), this.getAngleRad()});
// 	}

// 	public void resetLocalizataion() 
// 	{
// 		controller.resetLocalization();
// 		this.navx.reset();
// 	}


// 	public TrajectoryController getController()
// 	 {
// 		return this.controller;
// 	}

// 	public double getMaxA() 
// 	{
// 		return Math.min(Constants.leftMaxA, Constants.rightMaxA);
// 	}

// 	public double getMaxV() 
// 	{
// 		return Math.min(Constants.leftMaxV, Constants.rightMaxV);
// 	}

// 	protected void initialize(NavX navx, WPI_TalonSRX rightMotor, WPI_TalonSRX leftMotor, EncoderTalon3075 rightEncoder, EncoderTalon3075 leftEncoder) 
// 	{
// 		this.rightMotor = rightMotor;
// 		this.leftMotor = leftMotor;
// 		this.rightEncoder = rightEncoder;
// 		this.leftEncoder = leftEncoder;
// 		this.navx = navx;
// 		// this.setController(ControllerType.WangLi);
// 		this.rightEncoder.setPIDSourceType(PIDSourceType.kRate);
// 		this.leftEncoder.setPIDSourceType(PIDSourceType.kRate);
// 		// this.setLocalizataion(new Localizataion(rightEncoder, leftEncoder, navx));
// 		// this.localizataion.enable();
// 	}
// }
