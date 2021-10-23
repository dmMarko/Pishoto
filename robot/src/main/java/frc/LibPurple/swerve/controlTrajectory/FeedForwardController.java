// package frc.LibPurple.swerve.controlTrajectory;

// import frc.LibPurple.swerve.Swerve.SwerveDrivetrain;



// /**
//  * 
//  * @author 3075programming Our implementation of Motion Profiled control
//  */
// public class FeedForwardController extends TrajectoryController 
// {
// 	/**
// 	 * @param values      Motion Profiling values for generating the path
// 	 * @param motor       The motor to which we output the calculated value
// 	 * @param source      The sensor used for following the path
// 	 * @param profileType The Motion Profiling trajectory type
// 	 */

// 	public FeedForwardController(SwerveDrivetrain drivetrain)
// 	{
// 		super(drivetrain);
// 	}

// 	public void calculate(double passedTime, double dt)
// 	{
// 		// // if time is passed its will quit form trajectory
// 		// if(passedTime >= trajectory.getTotalTime())
// 		// {
// 		// 	this.setUnicycle(0, 0, 0);
// 		// 	this.disable();
// 		// 	return;
// 		// }
// 		// this.setpoint = trajectory.calculate(passedTime);
// 		// double[] nextSetpoint = trajectory.calculate(passedTime + period);
// 		// if (setpoint == null || nextSetpoint == null)
// 		// {	
// 		// 	Utils.printErr("[TrajectoryController] Error in Trajectory Calculate(), setpoint is null");
// 		// 	return;
// 		// }

// 		// // // SimpleMatrix matSetpoint = new SimpleMatrix(3, 1, true, setpoint);
// 		// // // SimpleMatrix matNextSetpoint = new SimpleMatrix(3, 1, true, nextSetpoint);
// 		// // Matrix matSetpoint = new Matrix(3, 1, setpoint);
// 		// // Matrix matNextSetpoint = new Matrix(3, 1, nextSetpoint);
// 		// // // matNextSetpoint.set(2, 0, matNextSetpoint.get(2, 0) * direction); //to spin
// 		// // // matNextSetpoint.set(1, 0, matNextSetpoint.get(1, 0) * direction);

// 		// // // SimpleMatrix dSetpoint = matNextSetpoint.minus(matSetpoint);
// 		// // Matrix dSetpoint = matNextSetpoint.minus(matSetpoint);
// 		// // // dSetpoint.set(0, 0, dSetpoint.get(0, 0) * direction);
// 		// // // dSetpoint.set(2, 0, dSetpoint.get(2, 0) * direction);


// 		// // double w = dSetpoint.get(2) / period;
// 		// // double v = (Math.sqrt(Math.pow(dSetpoint.get(0), 2) + Math.pow(dSetpoint.get(1), 2)) / period);

// 		// // if(v == 0){
// 		// // 	// Utils.print("x1 -x0 " + dSetpoint.get(0));
// 		// // 	// Utils.print("y1 - y0"+ dSetpoint.get(1));
// 		// // 	// Utils.print("dt"+ dt);
// 		// // 	Utils.print("avi");
// 		// // }

// 		// // v *= direction;
// 		// // w *= direction;
// 		// // this.velocity = v;

// 		// // // setUnicycle(1.5, 0);
// 		// // setUnicycle(v, w, 0);

// 		// Matrix matSetpoint = new Matrix(3, 1, setpoint);
// 		// Matrix matNextSetpoint = new Matrix(3, 1, nextSetpoint);

// 		// Matrix dSetpoint = matNextSetpoint.minus(matSetpoint);

// 		// double forward = dSetpoint.get(0) / period;
// 		// double strafe = dSetpoint.get(1) / period;
// 		// double rotation = dSetpoint.get(2) / period;

// 		// if(forward == 0 && strafe == 0 && rotation == 0){
// 		// 	Utils.print("avi");
// 		// }

// 		// forward *= direction;
// 		// strafe *= direction;
// 		// this.velocity = Math.sqrt(Math.pow(forward, 2) + Math.pow(strafe, 2)) * direction;

// 		// // setUnicycle(forward, strafe, rotation);
// 		// setUnicycle(forward, 0, 0);

// 	}

// 	@Override
// 	public double getSetpointVelocity() 
// 	{
// 		return this.velocity;
// 	}
// }