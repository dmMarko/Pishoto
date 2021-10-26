// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 first. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the first BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.DriveCommands;

// import java.io.FileWriter;
// import edu.wpi.first.wpilibj.command.Command;
// import frc.LibPurple.control.TrajectoryController;
// import frc.LibPurple.systems.DriveSystem3075;
// import frc.LibPurple.systems.DriveSystem3075.ControllerType;
// import frc.LibPurple.trajectories.ArcTrajectory;
// import frc.LibPurple.utils.Utils;

// public class DriveArc extends Command {
// 	FileWriter handle;
// 	DriveSystem3075 driveSystem;
// 	TrajectoryController controller;
// 	double radius;
// 	double ang;
// 	boolean clockWise;
// 	boolean debug;
// 	ControllerType typeController;

// 	public DriveArc(DriveSystem3075 driveSystem, double radius, double ang, boolean clockWise,
// 			ControllerType typeController, boolean debug) {
// 		requires(driveSystem);
// 		this.typeController = typeController;
// 		this.driveSystem = driveSystem;
// 		this.debug = debug;
// 		this.radius = radius;
// 		this.ang = ang;
// 		this.clockWise = clockWise;
// 		this.typeController = typeController;
// 	}

// 	@Override
// 	protected void initialize() {
// 		driveSystem.reset();
// 		driveSystem.reset();
// 		driveSystem.resetLocalizataion();
// 		driveSystem.setController(typeController);
// 		this.controller = driveSystem.getController();
// 		if (debug)
// 			this.handle = Utils.initializeCSVFile("/graphs/DriveArc");
// 		// Utils.addCSVLine(this.handle,
// 		// new String[] { "Time", "goal X", "real X", "goal Y", "real Y", "goal Angle", "real Angle", "goal Velocity", "real Velocity" });
// 		controller
// 				.setTrajectory(new ArcTrajectory(radius, ang, driveSystem.getMaxV(), driveSystem.getMaxA(), clockWise));
// 		this.controller.enable();
// 	}

// 	protected void execute() {
// 		if (this.debug) {
// 			double[] params = { controller.getPassedTime(), controller.getSetpoint()[0],
// 				driveSystem.getX(), controller.getSetpoint()[1],
// 				driveSystem.getY(), controller.getSetpoint()[2], driveSystem.getAngleRad(),
// 				controller.getSetpointVelocity(),
// 				(driveSystem.getLeftEncoder().getRate() + driveSystem.getRightEncoder().getRate()) / 2.0 };
// 			Utils.addCSVLine(this.handle, params);
// 		}
// 	}

// 	@Override
// 	protected boolean isFinished() {
// 		return false;
// 	}

// }
