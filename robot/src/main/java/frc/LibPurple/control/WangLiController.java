// package frc.LibPurple.control;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.LibPurple.math.Matrix;
// import frc.LibPurple.utils.Utils;
// import frc.robot.Constants;
// import frc.robot.Robot;

// public class WangLiController extends TrajectoryController 
// {

//     public WangLiController(WPI_TalonSRX motorRight, WPI_TalonSRX motorLeft)
//     {
//         super(motorRight, motorLeft);
//     }

//     @Override
//     public void calculate(double passedTime, double dt)
//     {

//         if(passedTime >= trajectory.getTotalTime())
// 		{
//             this.setUnicycle(0, 0);
// 			this.disable();
// 			return;
// 		}
//         setpoint = trajectory.calculate(passedTime + Constants.wangliHorizon);
//         // double[] nextSetpoint = trajectory.calculate(passedTime + Constants.wangliHorizon + period);
//         double[]  nextSetpoint = trajectory.getNextSetpoint();

//         if (setpoint == null || nextSetpoint == null) {
// 			Utils.printErr("[TrajectoryController] Error in Trajectory Calculate(), setpoint is null");
// 			return;
//         }
      
//         // SimpleMatrix matSetpoint = new SimpleMatrix(3, 1, true, setpoint);
//         // SimpleMatrix matNextSetpoint = new SimpleMatrix(3, 1, true, nextSetpoint);
//         Matrix matSetpoint = new Matrix(3, 1, setpoint);
// 		Matrix matNextSetpoint = new Matrix(3, 1, nextSetpoint);

//         // SimpleMatrix dSetpoint = matNextSetpoint.minus(matSetpoint);
//         // SimpleMatrix state = Robot.driveSystem.getStateMatrix();
//         Matrix dSetpoint = matNextSetpoint.minus(matSetpoint);
//         Matrix state = Robot.driveSystem.getStateMatrix();
//         // state.set(2, 0, direction * state.get(2, 0)); // flip the setpoint angle, cause it works for the WangLi

//         double robotAngle = Robot.driveSystem.getAngleRad();
//         // SimpleMatrix errorState = matSetpoint.minus(state);
//         Matrix errorState = matSetpoint.minus(state);
//         // SimpleMatrix rotationMat = new SimpleMatrix(new double[][] {{Math.cos(robotAngle), Math.sin(robotAngle), 0}, {-Math.sin(robotAngle), Math.cos(robotAngle), 0}, {0, 0, 1}});
//         Matrix rotationMat = new Matrix(new double[][] {{Math.cos(robotAngle), Math.sin(robotAngle), 0}, {-Math.sin(robotAngle), Math.cos(robotAngle), 0}, {0, 0, 1}});
//         // errorState = rotationMat.mult(errorState);
//         //need to mul

//         SmartDashboard.putNumber("Ex", (matSetpoint.get(0) - state.get(0))*Math.cos(robotAngle) + (matSetpoint.get(1) - state.get(1))*Math.sin(robotAngle));
//         SmartDashboard.putNumber("acrtan", Math.atan(errorState.get(0)));

//         // dSetpoint.set(0, 0, dSetpoint.get(0, 0) * direction);
//         // dSetpoint.set(2, 0, dSetpoint.get(2, 0) * direction);

//         double vr = Math.sqrt(Math.pow(dSetpoint.get(0), 2) + Math.pow(dSetpoint.get(1), 2)) / period;
//         vr *= direction;
//         // this.velocity = vr;
//         this.velocity = vr;
//         double wr = dSetpoint.get(2) / period;
//         // wr *= direction;
//         // double wr = dSetpoint.get(2) / dt * direction;


//         double v = (vr * Math.cos(errorState.get(2))) + (Constants.p1 * (2 / Math.PI) * Math.atan(errorState.get(0)));
//         // v *= direction;
//         double w = wr + (Constants.p2 * vr * errorState.get(1)) / (1 + Math.pow(errorState.get(0), 2) + Math.pow(errorState.get(1), 2)) + Constants.p3 * 2 / Math.PI * Math.atan(errorState.get(2));
//         // w *= direction;
//         this.setUnicycle(v, w);
//     }

//     @Override
//     public double getSetpointVelocity() 
//     {
//         return this.velocity;
//     }
// }