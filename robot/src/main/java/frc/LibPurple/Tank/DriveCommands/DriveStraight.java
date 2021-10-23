// package frc.LibPurple.DriveCommands;

// import java.io.FileWriter;
// import edu.wpi.first.wpilibj.command.Command;
// import frc.LibPurple.control.FFController;
// import frc.LibPurple.control.TrajectoryController;
// import frc.LibPurple.systems.DriveSystem3075;
// import frc.LibPurple.systems.DriveSystem3075.ControllerType;
// import frc.LibPurple.trajectories.StraightTrajectory;
// import frc.LibPurple.trajectories.StraightTrajectorySMP;
// import frc.LibPurple.utils.Utils;
// import frc.robot.Robot;

// public class DriveStraight extends Command 
// {
//     DriveSystem3075 driveSystem;
//     TrajectoryController controller;
//     ControllerType controllerType;

//     boolean endless;
    
// 	double distance;

// 	double maxA;
// 	double maxV;

// 	FileWriter handle;

// 	double tolerance;

// 	private boolean debug;


//     public DriveStraight(DriveSystem3075 driveSystem, double distance, ControllerType controllerType, boolean endless, boolean debug) 
//     {
// 		// requires(Robot.driveSystem);
// 		this.debug = debug;
// 		this.distance = distance;
// 		this.endless = endless;
//         this.driveSystem = driveSystem;
//         this.controllerType = controllerType;
//         this.maxA = this.driveSystem.getMaxA();
//         this.maxV = this.driveSystem.getMaxV();

// 	}

//     public DriveStraight(DriveSystem3075 driveSystem, double distance, double maxA, double maxV, ControllerType controllerType, boolean endless, boolean debug) 
//     {

// 		this(driveSystem, distance, controllerType, endless, debug);
// 		this.maxV = maxV;
// 		this.maxA = maxA;
//     }
    
// 	@Override
//     protected void initialize() 
//     {
//         driveSystem.reset();
//         driveSystem.resetLocalizataion();
//         if (debug)
// 		{
//             Utils.print("halo");
// 			this.handle = Utils.initializeCSVFile("/graphs/DriveSraight");
// 			// Utils.addCSVLine(this.handle, new String[] { "Time", "goal X", "real X", "goal Y", "real Y", "goal Angle",
// 					// "real Angle", "goal Velocity", "real Velocity" });
// 			Utils.addCSVLine(this.handle, new String[] {"Time", "TrajectoryLeftSpeed", "RealLeftSpeed", "Time", "TrajectoryRightSpeed", "RealRightSpeed" });
// 		}
//         driveSystem.setController(controllerType);
//         this.controller = driveSystem.getController();
//         this.controller.setTrajectory(new StraightTrajectorySMP(distance, maxA));
//         controller.setDirection(distance < 0);
//         // Robot.driveSystem.setInvertedDirection(distance < 0);
//         this.controller.enable();

// 	}

// 	@Override
//     protected void execute()
//     {
// 		if (debug) {
// 			double[] params = {controller.getPassedTime(), controller.getLeftSpeed(), Robot.driveSystem.getLeftEncoder().getRate(), controller.getPassedTime(), controller.getRightSpeed(), Robot.driveSystem.getLeftEncoder().getRate()};
// 			// double[] params = {leftMP.getPassedTime(),rightMP.getPassedTime()};
// 			Utils.addCSVLine(this.handle, params);
// 		}
// 	}

// 	@Override
//     public boolean isFinished() 
//     {
// 		return false;
// 	}

// 	@Override
//     protected void end()
//     {
//         this.controller.disable();
//     }

// 	@Override
//     protected void interrupted() 
//     {
//         end();
//         driveSystem.set(0, 0);
// 	}
// }
