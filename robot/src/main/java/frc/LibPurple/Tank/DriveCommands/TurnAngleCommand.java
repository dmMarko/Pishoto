// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 first. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the first BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.DriveCommands;

// import com.sun.org.apache.xalan.internal.templates.Constants;
// import com.sun.org.apache.xml.internal.serializer.utils.Utils;

// import edu.wpi.first.wpilibj.command.Command;
// import frc.LibPurple.control.FFController;
// import frc.LibPurple.systems.DriveSystem3075;
// import frc.LibPurple.systems.DriveSystem3075.Controller;
// import frc.LibPurple.trajectories.TrajectoryTrapezoidal;
// import frc.robot.Robot;

// public class TurnAngleCommand extends Command {
//   DriveSystem3075 driveSystem;
// 	FFController rightVelocityPID;
// 	FFController leftVelocityPID;

// 	boolean endless;
// 	double leftDistance;
// 	double rightDistance;
// 	double angle;
// 	double maxA;
// 	double maxV;

// 	Controller prevState;
// 	private Type Type;

// 	FileWriter handle;

// 	public TurnAngleCommand(DriveSystem3075 driveSystem, double angle, boolean endless, double maxA) {
// 		requires(Robot.driveSystem);

// 		this.driveSystem = driveSystem;
// 		this.leftDistance = driveSystem.distancePerAngle * angle;
// 		this.rightDistance = -driveSystem.distancePerAngle * angle;
// 		this.angle = angle;
// 		this.endless = endless;
// 		this.maxA = maxA;

// 		rightVelocityPID = driveSystem.getRightVelocityPIDController();
// 		leftVelocityPID = driveSystem.getLeftVelocityPIDController();
// 	}

// 	public TurnAngle(DriveSystem3075 driveSystem, double angle, boolean endless) {
// 		requires(driveSystem);

// 		this.driveSystem = driveSystem;
// 		this.leftDistance = Constants.distancePerAngle * angle;
// 		this.rightDistance = -Constants.distancePerAngle * angle;
// 		this.angle = angle;
// 		this.endless = endless;
// 		this.maxA = Math.min(driveSystem.getLeftTurnMaxA(), driveSystem.getRightTurnMaxA());
// 		this.maxV = Math.min(driveSystem.getLeftTurnMaxA(), driveSystem.getRightTurnMaxV());
// 		this.rightVelocityPID = driveSystem.getRightVelocityPIDController();
// 		this.leftVelocityPID = driveSystem.getLeftVelocityPIDController();
// 	}

// 	@Override
// 	protected void initialize() {
// 		prevState = driveSystem.state;
// 		driveSystem.reset();

// 		driveSystem.setTurnVelocityPIDValues(driveSystem.leftTurnVelocityPIDValue,
// 				driveSystem.rightTurnVelocityPIDValue);

// 		driveSystem.rightVelocityPID.setTrajectory(new TrajectoryTrapezoidal(rightDistance, maxA, maxV));
// 		driveSystem.leftVelocityPID.setTrajectory(new TrajectoryTrapezoidal(leftDistance, maxA, maxV));
// 		driveSystem.enterState(DriveSystem3075.Controller.FeedForward);
// 		this.handle = Utils.initialiseCSVFile("/graphs/turn", 7);
// 		if (this.handle == null)
// 			Utils.print("Error opening file");
// 	}

// 	@Override
// 	protected void execute() {
// 		double[] params = { leftVelocityPID.getPassedTime(), -leftVelocityPID.getSetpoint().velocity,
// 				Robot.driveSystem.getLeftEncoder().getRate(), rightVelocityPID.getPassedTime(),
// 				rightVelocityPID.getSetpoint().velocity, Robot.driveSystem.getRightEncoder().getRate() };
// 		Utils.addCSVLine(this.handle, params);

// 	}

// 	@Override
// 	public boolean isFinished() {
// 		if (endless) {
// 			return false;
// 		}
// 		return Utils.inRange(driveSystem.getAngle(), this.angle, driveSystem.angleTolerance)
// 				&& this.rightVelocityPID.isTimeUp();
// 	}

// 	@Override
// 	protected void end() {
// 		// Utils.closeCSVFile(this.handle);
// 		leftVelocityPID.disable();
// 		rightVelocityPID.disable();
// 		driveSystem.enterState(this.prevState);
// 	}

// 	@Override
// 	protected void interrupted() {
// 		end();
// 	}
// }
