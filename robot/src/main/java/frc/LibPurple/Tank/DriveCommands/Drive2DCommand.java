// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 first. All Rights Reserved.                              */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the first BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.DriveCommands;

// import java.io.FileWriter;

// import edu.wpi.first.wpilibj.command.Command;
// import frc.LibPurple.control.PIDvalue;
// import frc.LibPurple.control.TrajectoryController;
// import frc.LibPurple.systems.DriveSystem3075;
// import frc.LibPurple.systems.DriveSystem3075.ControllerType;
// import frc.LibPurple.trajectories.TrajectoryFile;
// import frc.LibPurple.utils.Utils;
// import frc.robot.Robot;

// public class Drive2DCommand extends Command 
// {
// 	DriveSystem3075 driveSystem;
// 	TrajectoryController controller;
// 	PIDvalue rightVelocityPIDValue;
// 	PIDvalue leftVelocityPIDValue;
// 	boolean reversed;
// 	boolean debug;
// 	String fileName;

// 	ControllerType controllerType;

// 	FileWriter handle;

// 	public Drive2DCommand(DriveSystem3075 driveSystem, String fileName, boolean reversed,
// 			ControllerType controllerType, boolean debug) {
// 		requires(Robot.driveSystem);
// 		this.debug = debug;
// 		this.driveSystem = driveSystem;
// 		this.fileName = fileName;
// 		this.controllerType = controllerType;
// 		this.reversed = reversed;

// 		// Robot.trajectoryFile = new TrajectoryFile(this.fileName, this.reversed);
// 	}

// 	@Override
// 	protected void initialize() {
// 		driveSystem.reset();
// 		if (debug)
// 		{
// 			this.handle = Utils.initializeCSVFile("/graphs/Bezier");
// 			// Utils.addCSVLine(this.handle, new String[] { "Time", "goal X", "real X", "goal Y", "real Y", "goal Angle",
// 			// "real Angle", "goal Velocity", "real Velocity" });
// 			Utils.addCSVLine(this.handle, new String[] {"Time", "TrajectoryLeftSpeed", "RealLeftSpeed", "Time", "TrajectoryRightSpeed", "RealRightSpeed" });
// 		}	
// 		// this.handle = Robot.handle;
// 		this.driveSystem.setController(controllerType);
// 		this.controller = driveSystem.getController();
// 		controller.setTrajectory(new TrajectoryFile(this.fileName, reversed));
// 		// controller.setTrajectory(Robot.trajectoryFile);
// 		controller.setDirection(reversed);
// 		driveSystem.resetLocalizataion();
// 		// Robot.driveSystem.setInvertedDirection(reversed);
// 		this.controller.enable();
// 	}

// 	protected void execute()
// 	 {
// 		// double[] params = { controller.getPassedTime(), controller.getSetpoint()[0],
// 		// 		driveSystem.getX(), controller.getSetpoint()[1],
// 		// 		driveSystem.getY(), controller.getSetpoint()[2], driveSystem.getAngleRad(),
// 		// 		controller.getSetpointVelocity(),
// 		// 		(driveSystem.getLeftEncoder().getRate() + driveSystem.getRightEncoder().getRate()) / 2.0 };
// 		double[] params = { controller.getPassedTime(), controller.getSetpoint()[0], controller.getSetpoint()[1]};
// 		// double[] params = {controller.getPassedTime(), controller.getLeftSpeed(), Robot.driveSystem.getLeftEncoder().getRate(), controller.getPassedTime(), controller.getRightSpeed(), Robot.driveSystem.getLeftEncoder().getRate()};
// 		if(debug)
// 			Utils.addCSVLine(this.handle, params);
// 	}

// 	@Override
// 	protected void end() {
// 		// Utils.closeCSVFile(this.handle);[]\
// 		controller.disable();
// 		Utils.print("2D Done!");
// 	}

// 	@Override
// 	protected void interrupted() {
// 		end();
// 	}

// 	@Override
// 	protected boolean isFinished() {
// 		// TODO Auto-generated method stub
// 		return false;
// 		// return controller.getPassedTime() == controller.getTrajectory().getTotalTime();
// 	}
// }
