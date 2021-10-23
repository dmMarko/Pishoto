// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.swerve.controlTrajectory;

// import edu.wpi.first.wpilibj.Timer;
// import frc.LibPurple.control.Loop;
// import frc.LibPurple.math.RigidTransform2D;
// import frc.LibPurple.math.Rotation2D;
// import frc.LibPurple.math.Vector2D;
// import frc.LibPurple.swerve.Swerve.SwerveDrivetrain;
// import frc.LibPurple.swerve.Swerve.SwerveModule;


// /**
//  * Add your docs here.
//  */
// public class KinematicsPos extends Loop{
//     protected double startTime;
// 	protected double passedTime;
// 	protected double now;
// 	protected double dt;
//     protected double lastTime;
//     protected Vector2D pos = Vector2D.ZERO;
//     protected double angle = 0;
//     protected SwerveDrivetrain drivetrain;
    
//     public KinematicsPos(SwerveDrivetrain drivetrain){
//         super();
//         this.drivetrain = drivetrain;
//     }

//     @Override
//     public void run(){
//         if(!enabled){
//             disable();
//             return;
//         }
//     }

//     public void calculate(){
//         now = Timer.getFPGATimestamp();
//         passedTime = now - startTime;

//         double robotRotation = Math.toRadians(drivetrain.navX.getYaw());
//         this.angle = Math.toDegrees(robotRotation);
        
//         Vector2D averageCenter = Vector2D.ZERO;
//         for(SwerveModule module : drivetrain.getSwerveModules()){
//             module.updateSensors();
//             module.updateKinematics(robotRotation);

//             Vector2D estimatedCenter = new RigidTransform2D(module.getCurrentPos(),
//                     Rotation2D.fromRadians(robotRotation))
//                     .transformBy(new RigidTransform2D(module.getModulePos().inverse(), Rotation2D.ZERO)).translation;

//             averageCenter = averageCenter.add(estimatedCenter);
//         }
//         averageCenter = averageCenter.scale(1.0 / drivetrain.getSwerveModules().length);

//         pos = averageCenter;


//         lastTime = passedTime;
//     }

//     public void enable()
// 	{
// 		super.enable();
// 		startTime = Timer.getFPGATimestamp();
// 	}
	
// 	public double getPassedTime()
// 	{
// 		return this.passedTime;
//     }

//     public double getAngle()
//     {
//         return this.angle;
// 	}
	
// 	public double getAngleRad()
// 	{
// 		return Math.toRadians(this.angle);
//     }
    
//     public Vector2D getPos(){
//         return pos;
//     }

//     public void reset()
//     {
//         this.pos = Vector2D.ZERO;
//         this.now = 0;
//         this.passedTime = 0;
//         this.lastTime = 0;
//         this.startTime = 0;
// 	}
// }
