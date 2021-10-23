// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 first. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the first BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.DriveCommands;

// import java.io.FileWriter;

// import edu.wpi.first.wpilibj.command.Command;
// import frc.LibPurple.control.FFController;
// import frc.LibPurple.control.PIDvalue;
// import frc.LibPurple.control.TrajectoryController;
// import frc.LibPurple.systems.DriveSystem3075.Controller;
// import frc.LibPurple.trajectories.OneSideTrajectory;
// import frc.robot.Robot;

// public class DriveDistanceCommand extends Command 
// {
// 	FFController rightVelocityPID;
// 	FFController leftVelocityPID;

// 	boolean endless;
// 	double leftDistance;
// 	double rightDistance;

// 	double leftMaxA, rightMaxA;
// 	double leftMaxV, rightMaxV;

// 	Controller prevState;

// 	OneSideTrajectory.Type Type;

// 	FileWriter handle;

// 	double tolerance;
// 	PIDvalue arcValues;

// 	boolean isGyro = false;
// 	double leftRadius, rightRadius;
// 	boolean clockwise;

// 	double startV;
// 	double endV;

// 	boolean integrated = false;
// 	private boolean debug;


// 	public DriveDistanceCommand(double leftDistance, double rightDistance, boolean endless,
// 		 double leftmaxA, double rightMaxA,TrajectoryController controller, boolean debug) {
// 		requires(Robot.driveSystem);
// 		this.debug = debug;
// 		this.leftDistance = leftDistance;
// 		this.rightDistance = rightDistance;

// 		this.leftMaxA = leftmaxA;
// 		this.rightMaxA = rightMaxA;
// 		this.endless = endless;

// 		this.startV = 0;
// 		this.endV = 0;
// 		this.rightVelocityPID = driveSystem.getRightVelocityPIDController();
// 		this.leftVelocityPID = driveSystem.getLeftVelocityPIDController();

// 	}

// 	public DriveDistanceCommand(DriveSystem3075 driveSystem, double leftDistance, double rightDistance, boolean endless,
// 			double leftmaxA, double rightMaxA, Type Type, double leftMaxV, double rightMaxV, TrajectoryController controller, boolean debug) {

// 		this(driveSystem, leftDistance, rightDistance, endless, leftmaxA, rightMaxA, controller, debug);
// 		this.Type = Type;
// 		this.leftMaxV = leftMaxV;
// 		this.rightMaxV = rightMaxV;
// 	}

// 	@Override
// 	protected void initialize() {
// 		prevState = driveSystem.state;
// 		driveSystem.reset();

// 		driveSystem.setPIDValues(Constants.leftVelocityPID, Constants.rightVelocityPID);

// 		driveSystem.leftVelocityPID.setTrajectory(new TrajectoryTrapezoidal(leftDistance, leftMaxA, leftMaxV));
// 		driveSystem.rightVelocityPID.setTrajectory(new TrajectoryTrapezoidal(rightDistance, rightMaxA, rightMaxV));

// 		driveSystem.enterState(DriveSystem3075.Controller.FeedForward);
// 		if (debug) {
// 			this.handle = Utils.initialiseCSVFile("/graphs/DriveArc");
// 			if (this.handle == null)
// 				Utils.print("Error opening file");
// 		}
// 	}

// 	@Override
// 	protected void execute() {
// 		if (debug) {
// 			double[] params = { leftVelocityPID.getPassedTime(), leftVelocityPID.getSetpoint().velocity,
// 					Robot.driveSystem.getLeftEncoder().getRate(), rightVelocityPID.getPassedTime(),
// 					rightVelocityPID.getSetpoint().velocity, Robot.driveSystem.getRightEncoder().getRate() };
// 			// double[] params = {leftMP.getPassedTime(),rightMP.getPassedTime()};
// 			Utils.addCSVLine(this.handle, params);
// 		}
// 	}

// 	@Override
// 	public boolean isFinished() {
// 		return false;
// 	}

// 	@Override
// 	protected void end() {
// 		// Utils.closeCSVFile(this.handle);
// 		leftVelocityPID.disable();
// 		rightVelocityPID.disable();
// 		// Utils.print("left end velocity: " +
// 		// Robot.driveSystem.getLeftEncoder().getRate());
// 		driveSystem.enterState(this.prevState);
// 	}

// 	@Override
// 	protected void interrupted() {
// 		end();
// 	}
// }
