// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.swerve.Swerve;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.LibPurple.control.PIDvalue;
// import frc.LibPurple.math.RigidTransform2D;
// import frc.LibPurple.math.Rotation2D;
// import frc.LibPurple.math.Vector2D;
// import frc.LibPurple.sensors.NavX;
// import frc.LibPurple.swerve.Control.HolonomicFeedForward;
// import frc.LibPurple.swerve.SDriveCommands.HolonomicDriveCommand;
// import frc.LibPurple.swerve.SDriveCommands.SDrive2DCommand;
// import frc.LibPurple.swerve.SDriveCommands.SDriveStraight;
// import frc.LibPurple.swerve.controlTrajectory.FeedForwardController;
// import frc.LibPurple.swerve.controlTrajectory.MotionProfile;
// import frc.LibPurple.swerve.controlTrajectory.PurePursuitController;
// import frc.LibPurple.swerve.controlTrajectory.TrajectoryController;
// import frc.LibPurple.utils.Utils;
// import frc.robot.Constants;


// /**
//  * Add your docs here.
//  */
// public abstract class SwerveDrivetrain extends HolonomicDrivetrain{
//     public static enum ControllerType{
//         FeedForward, MotionProfile, PurePursuit;
//     }

//     protected PIDvalue forwardValueMotionProfile = Constants.forwardValueMotionProfile;
//     protected PIDvalue strafeValueMotionProfile = Constants.strafeValueMotionProfile;
//     protected PIDvalue rotationValueMotionProfile = Constants.rotationValueMotionProfile;

//     protected HolonomicFeedForward feedForwardHolonomic = new HolonomicFeedForward(Constants.forwardFeedForwardHolonomicConstants,
//         Constants.starfeFeedForwardHolonomicConstants);

//     protected TrajectoryController controller;

//     // public static AHRS navX = new AHRS(RobotMap.navXPort, (byte) 200);
//     public static NavX navX = new NavX();
//     private Vector2D kinematicPosition = Vector2D.ZERO;
//     private Vector2D kinematicVelocity = Vector2D.ZERO;
//     private double lastKinematicTimestamp;
//     private Rotation2D adjustmentAngle = Rotation2D.ZERO;
//     // private Rotation2D unAdjustedAngle = new Rotation2D(0, 0, false);
//     // private Rotation2D unAdjustedAngle = Rotation2D.fromDegrees(Robot.navX.getYaw());
//     private Rotation2D unAdjustedAngle;
//     private Rotation2D angle;

//     public void holonomicDrive(Vector2D translation, double rotation, boolean fieldOriented) {
//         // Utils.print("hallo");
//         if (fieldOriented) {
//             unAdjustedAngle = Rotation2D.fromDegrees(-navX.getYaw());
//             angle = unAdjustedAngle.rotateBy(adjustmentAngle);
//             translation = translation.rotateBy(angle);
//         }

//         int direction = 1;

//         // for (SwerveModule module : getSwerveModules()) {
//         //     Vector2D velocity = module.getModulePos().normal().scale(rotation).add(translation);

//         //     module.setTargetVelocity(velocity);
//         // }

//         for(int i=0; i < getSwerveModules().length; i++){
//             double a;
//             if(i == 0)
//             {
//                 a = Math.PI / 4;
//             }
//             else if(i == 1)
//             {
//                 a = Math.toRadians(135);
//             }
//             else if(i == 2)
//             {
//                 a = Math.toRadians(315);
//             }
//             else
//             {
//                 a = Math.toRadians(225);
//             }
//             // a = Math.PI / 4;
//             double x = rotation * Math.cos(a * direction);
//             double y = rotation * Math.sin(a * direction);
            
//             Vector2D Vr = new Vector2D(x, y);
//             Vector2D velocity = (translation.add(Vr));

//             getSwerveModules()[i].setTargetVelocity(velocity);
//     }
//         }
    

//     public abstract SwerveModule[] getSwerveModules();

   
//     public void stop() {
//         holonomicDrive(Vector2D.ZERO, 0, false);
//     }

//     @Override
//     public synchronized void updateKinematics(double timestamp) {
//         double robotRotation = Math.toRadians(navX.getYaw());
//         double dt = timestamp - lastKinematicTimestamp;
//         lastKinematicTimestamp = timestamp;

//         SwerveModule[] swerveModules = getSwerveModules();

//         Vector2D averageCenter = Vector2D.ZERO;
//         for (SwerveModule module : swerveModules) {
//             module.updateSensors();
//             module.updateKinematics(robotRotation);

//             Vector2D estimatedCenter = new RigidTransform2D(module.getCurrentPos(),
//                     Rotation2D.fromRadians(robotRotation))
//                     .transformBy(new RigidTransform2D(module.getModulePos().inverse(), Rotation2D.ZERO)).translation;

//             averageCenter = averageCenter.add(estimatedCenter);
//         }
//         averageCenter = averageCenter.scale(1.0 / swerveModules.length);

//         kinematicVelocity = averageCenter.subtract(kinematicPosition).scale(1.0 / dt);
//         kinematicPosition = averageCenter;

//         for (SwerveModule module : swerveModules) {
//             module.resetKinematics(new RigidTransform2D(kinematicPosition, Rotation2D.fromRadians(robotRotation))
//                     .transformBy(new RigidTransform2D(module.getModulePos(), Rotation2D.ZERO)).translation);
//             module.updateState(dt);
//         }
//     }

//     @Override
//     public synchronized void updateKinematicsAuto(double timestamp) {
//         double robotRotation = Math.toRadians(navX.getYaw());
//         double dt = timestamp - lastKinematicTimestamp;
        
//         SwerveModule[] swerveModules = getSwerveModules();
        
//         Vector2D averageCenter = Vector2D.ZERO;
//         for (SwerveModule module : swerveModules) {
//             module.updateSensors();
//             module.updateKinematics(robotRotation);
            
//             Vector2D estimatedCenter = new RigidTransform2D(module.getCurrentPos(),
//             Rotation2D.fromRadians(robotRotation))
//             .transformBy(new RigidTransform2D(module.getModulePos().inverse(), Rotation2D.ZERO)).translation;
            
//             averageCenter = averageCenter.add(estimatedCenter);
//         }
//         averageCenter = averageCenter.scale(1.0 / swerveModules.length);
        
//         kinematicVelocity = averageCenter.subtract(kinematicPosition).scale(1 / (timestamp - lastKinematicTimestamp));
//         kinematicPosition = averageCenter;
//         lastKinematicTimestamp = timestamp;

//         for (SwerveModule module : swerveModules) {
//             module.resetKinematics(new RigidTransform2D(kinematicPosition, Rotation2D.fromRadians(robotRotation))
//                     .transformBy(new RigidTransform2D(module.getModulePos(), Rotation2D.ZERO)).translation);
//             module.updateStateAuto(dt);
//         }
//     }
   
//     // @Deprecated
//     public synchronized void resetKinematics(double timestamp) {
//         resetKinematics(Vector2D.ZERO, timestamp);
//         for (SwerveModule module : getSwerveModules()){
//             module.resetKinematics();
//         }
//     }

//     public synchronized void resetKinematics(Vector2D position, double timestamp) {
//         for (SwerveModule module : getSwerveModules()) {
//             // module.resetKinematics(position.add(module.getModulePos()));
//             module.resetKinematics(position);
//         }

//         kinematicVelocity = Vector2D.ZERO;
//         kinematicPosition = position;
//         lastKinematicTimestamp = timestamp;
//     }

//     @Override
//     public Vector2D getKinematicPosition() {
//         return kinematicPosition;
//     }

//     @Override
// 	public Vector2D getKinematicVelocity() {
// 		return kinematicVelocity;
//     }

//     public Rotation2D getAngle(){
//         return Rotation2D.fromDegrees(-navX.getYaw());
//     }

//     public void setZeroPower(){
//         for(SwerveModule module : getSwerveModules()){
//             module.setDriveOutput(0);
//             module.setAngleOutput(0);;
//         }
//     }

//     public void resetPID(){
//         for(SwerveModule module : getSwerveModules()){
//             module.resetPID();
//         }
//     }

//     @Override
//     public void periodic() {
//         // TODO Auto-generated method stub
//         super.periodic();
//     }
    
//     public abstract void resetEncoders();

// 	public void setController(ControllerType controller) {
// 		if(controller == ControllerType.FeedForward)
// 		{
// 			if(!(this.controller instanceof FeedForwardController)){
//                 this.controller = new FeedForwardController(this);
//             }
//         }
//         if(controller == ControllerType.PurePursuit){
//             if(!(this.controller instanceof PurePursuitController)){
//                 this.controller = new PurePursuitController(this, rotationValueMotionProfile, feedForwardHolonomic);
//             }
//         }
//         if(controller == ControllerType.MotionProfile){
//             if(!(this.controller instanceof MotionProfile)){
//                 this.controller = new MotionProfile(this, forwardValueMotionProfile, strafeValueMotionProfile,
//                     rotationValueMotionProfile, feedForwardHolonomic);
//             }
//         }
// 	}

// 	public TrajectoryController getController() {
// 		return this.controller;
//     }

//     public CommandBase SDriveStraight(double distance, double angle, double rotation, ControllerType controllerType, boolean endless, boolean debug)
// 	{
// 		return new SDriveStraight(this, distance, angle, rotation, controllerType, endless, debug);
// 	}

// 	public CommandBase SDiveStraight(double distance, double angle, double rotation, double maxA, double maxV, ControllerType controllerType, boolean endless, boolean debug)
// 	{
// 		return new SDriveStraight(this, distance, angle, rotation, maxA, maxV, controllerType, endless, debug);
// 	}

// 	// public Command driveArc(double radius, double ang, boolean clockWise, ControllerType controller, boolean debug)
// 	// {
// 	// 	return new DriveArc(this, radius, ang, clockWise, controller, debug);
// 	// }

//     public CommandBase SDrive2DCommand(String name, boolean reversed, ControllerType controllerType, boolean debug){
//         return new SDrive2DCommand(this, "/home/lvuser/deploy/" + name + ".csv", reversed, controllerType, debug);
//     }
// }

