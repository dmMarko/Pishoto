// package frc.LibPurple.control;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import frc.LibPurple.utils.Utils;



// /**
//  * 
//  * @author 3075programming Our implementation of Motion Profiled control
//  */
// public class ConstController extends TrajectoryController 
// {
//     private double constV;

// 	/**
// 	 * @param values      Motion Profiling values for generating the path
// 	 * @param motor       The motor to which we output the calculated value
// 	 * @param source      The sensor used for following the path
// 	 * @param profileType The Motion Profiling trajectory type
// 	 */

// 	public ConstController(WPI_TalonSRX motorRight, WPI_TalonSRX motorLeft, double constV)
// 	{
//         super(motorRight, motorLeft);
//         this.constV = constV;
// 	}

// 	public void calculate(double passedTime, double dt) 
// 	{
// 		// if time is passed its will quit form trajectory
// 		if(passedTime >= trajectory.getTotalTime())
// 		{
// 			this.disable();
// 			return;
// 		}
// 		this.setpoint = trajectory.calculate(passedTime);
// 		double[] nextSetpoint = trajectory.calculate(passedTime + dt);
// 		if (setpoint == null || nextSetpoint == null)
// 		{	
// 			Utils.printErr("[TrajectoryController] Error in Trajectory Calculate(), setpoint is null");
// 			return;
// 		}

// 		setUnicycle(this.constV, 0);

// 	}

// 	@Override
// 	public double getSetpointVelocity() 
// 	{
// 		return this.constV;
// 	}
// }