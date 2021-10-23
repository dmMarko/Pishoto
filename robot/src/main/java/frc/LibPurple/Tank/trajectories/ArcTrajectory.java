package frc.LibPurple.Tank.trajectories;

// import frc.robot.Constants;
// public class ArcTrajectory extends StraightTrajectory
// {
//     double clockWise;
//     double radius;
//     public ArcTrajectory(double radius, double deg, double maxV, double maxA, boolean clockWise)
//     {
//         super((radius + Constants.robotWidth / 2.0) * 2.0 * Math.PI * deg / 360, maxV, maxA);
//         this.radius = radius;
//         this.clockWise = clockWise ? 1 : -1;
    
//     }
//     public double[] calculate(double time)
//     {
//         double arcDistance = super.calculate(time)[0];
//         double radian = arcDistance / (radius + Constants.robotWidth / 2.0);
//         this.setpoint[0] = radius * Math.cos(radian) * clockWise;
//         this.setpoint[1] = radius * Math.sin(radian);
//         this.setpoint[2] = radian;
//         return this.setpoint;

//     }
// }