// package frc.LibPurple.control;

// import frc.LibPurple.math.Matrix;
// import frc.LibPurple.utils.Utils;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import frc.robot.Constants;
// import frc.robot.Robot;
// // import org.ejml.simple.SimpleMatrix;

// public class PurePursuitController extends TrajectoryController
// {

//     protected double epislon = 0.000000000000001;
//     protected double leftSpeed;
//     protected double rightSpeed;

//     public PurePursuitController(WPI_TalonSRX motorRight, WPI_TalonSRX motorLeft)
//     {
//         super(motorRight, motorLeft);
//     }

//     @Override
//     public void calculate(double passedTime, double dt) 
//     {
        
//         // now = Timer.getFPGATimestamp();
//     	// passedTime = now - startTime;
//         // double dt = passedTime - lastTime;    
//         if(trajectory.getTotalTime() < passedTime)
//         {
//             return;
//         }
//     	// Indication if on target or trajectory time has passed
        
//         double[] goalSetPoint = trajectory.calculate(passedTime + Constants.purePursuitHorizon);
        
//         if (goalSetPoint == null) {
//             Utils.printErr("[TrajectoryController] Error in Trajectory Calculate(), setpoint is null");
//             return;
//         }


//         // SimpleMatrix matSetpoint = new SimpleMatrix(3, 1, true, trajectory.calculate(passedTime));
// 		// SimpleMatrix matNextSetpoint = new SimpleMatrix(3, 1, true, trajectory.calculate(passedTime + dt));
//         // SimpleMatrix dSetpoint = matNextSetpoint.minus(matSetpoint);
//         Matrix matSetpoint = new Matrix(3, 1, trajectory.calculate(passedTime));
// 		Matrix matNextSetpoint = new Matrix(3, 1, trajectory.calculate(passedTime + dt));
//         Matrix dSetpoint = matNextSetpoint.minus(matSetpoint);

//         double goalX = goalSetPoint[0];
//         double goalY = goalSetPoint[1];
//         double realY = Robot.driveSystem.getY();
//         double realX = Robot.driveSystem.getX();
//         double realA = Robot.driveSystem.getAngleRad();
//         // double fix = (-1 / Math.tan(realA) * goalX - goalY + realX / Math.tan(realA) + realY) / Math.sqrt(Math.pow(-1 / Math.tan(realA), 2) + 1);
//         // double clockWise = Math.sin(realA - Math.atan2((goalY - realY), (goalX - realX)));
//         // double clockWise = (Math.tan(realA) * goalX + (realY - Math.tan(realA) * realX) - goalY);
//         // if (goalX == realX)
//         //     goalX += epislon;
//         // double b = ((goalX - realX) * (2.0 * realY * Math.tan(realA) - (goalX - realX)) - goalY * goalY + realX * realX) / (2.0 * (realY - goalY) + 2.0 * Math.tan(realA) * (goalX - realX));
//         // double a = (2.0 * b * (realY - goalY) +  goalX * goalX + goalY * goalY - realX * realX - realY * realY) / (2.0 * (goalX - realX));
//         // double radius = Math.sqrt(Math.pow(realX - a, 2) + Math.pow(realY - b, 2));
//         // if (Double.isNaN(radius))
//         //     radius = Double.MAX_VALUE;
//         double deltaX = (goalY - realY) * Math.sin(realA) + (goalX - realX) * Math.cos(realA);
//         double deltaY = (goalY - realY) * Math.cos(realA) - (goalX - realX) * Math.sin(realA);
//         Utils.print(deltaY + "");
//         if (deltaX == 0)
//             deltaX = epislon;
//         double radius = (deltaX * deltaX + deltaY * deltaY) / (2 *deltaX * deltaX);
//         this.velocity = (Math.sqrt(Math.pow(dSetpoint.get(0), 2) + Math.pow(dSetpoint.get(1), 2))) / dt;
//         if (deltaX > 0)
//         {
//             double leftRadius = radius - Constants.robotWidth / 2.0;
//             double rightRadius = radius + Constants.robotWidth / 2.0;
//             double ratio = leftRadius / rightRadius;
//             this.rightSpeed = 2.0 * velocity / (1.0 + ratio);
//             this.leftSpeed = ratio * rightSpeed; 
//         }
//         else
//         {
//             double leftRadius = radius + Constants.robotWidth / 2.0;
//             double rightRadius = radius - Constants.robotWidth / 2.0;
//             double ratio = rightRadius / leftRadius;
//             this.leftSpeed = 2.0 * velocity / (1.0 + ratio);
//             this.rightSpeed = ratio * leftSpeed; 
//         }
        
//         double w = (2.0 * leftSpeed - 2.0 * velocity) / Constants.robotWidth;
//         this.setUnicycle(velocity, w);
//     }

//     @Override
//     public double getSetpointVelocity()
//     {
//         return this.velocity;
//     }
// }