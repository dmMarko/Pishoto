// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.swerve.Swerve;

// import com.revrobotics.CANEncoder;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.EncoderType;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.LibPurple.control.PIDController;
// import frc.LibPurple.control.PIDvalue;
// import frc.LibPurple.math.Vector2D;
// import frc.robot.Constants;

// /**
//  * Add your docs here.
//  */
// public class SwerveModule3075 extends SwerveModule{
//     private PIDvalue angleValue = new PIDvalue(0.3, 0.0001, 0.001);
//     private PIDvalue driveValue = new PIDvalue(0.001, 0, 0);
//     private PIDController anglController = new PIDController(angleValue);
//     private PIDController driveController = new PIDController(driveValue);
//     private final double angleOffset;
    
//     public CANSparkMax driveMotor, angleMotor;
//     private CANEncoder driveEncoder;
//     private CANEncoder dontUseEncoder;
//     private double driveEncoderTicks;
//     private double driveVelocityRpm;
//     private double driveCurrent;
//     private double inverted = 1.0;

//     private double distancePerPulse;
    
//     public SwerveModule3075(Vector2D modulePos, double angleOffset, CANSparkMax angleMotor,
//      CANSparkMax driveMotor, boolean motorPhase, boolean invertEncoder, double distancePerPulse){
//         super(modulePos);
//         this.angleOffset = angleOffset;
//         this.angleMotor = angleMotor;
//         this.driveMotor = driveMotor;
//         // this.angleEncoder = angleEncoder;
//         this.driveEncoder = driveMotor.getEncoder(EncoderType.kHallSensor, 42);
//         // this.driveEncoder = new CANEncoder(this.driveMotor);
//         this.dontUseEncoder = new CANEncoder(this.angleMotor);

//         driveEncoderTicks = driveEncoder.getPosition();
//         driveVelocityRpm = driveEncoder.getVelocity();
//         driveCurrent = driveMotor.getOutputCurrent();
//         // this.angleEncoder = new CANAnalog(angleMotor, AnalogMode.kAbsolute);

//         driveMotor.setSmartCurrentLimit(80);
//         driveMotor.setInverted(motorPhase);

//         anglController.setInputRange(0.0, 2.0 * Math.PI);
//         anglController.setContinuous(true);
//         anglController.setOutputRange(-0.5, 0.5);


//         driveEncoder.setPositionConversionFactor(42);
//         driveEncoder.setVelocityConversionFactor(42);
//         // driveEncoder.setInverted(invertEncoder);
//         if(invertEncoder){
//             inverted = -inverted;
//         }

//         this.distancePerPulse = distancePerPulse;
//     }

//     @Override
//     public double getAngle() {
//         return Constants.getModuleAngle(getModuleNumber());
//     }

//     @Override
//     public void setVelocity(double velocity){
//         SmartDashboard.putNumber("key", velocity);
//         driveController.setSetpoint(velocity);
//     }

//     public double getVelocity(){
//         return driveEncoder.getVelocity();
//     }

//     public double getAngleDontUse(){
//         return dontUseEncoder.getPosition();
//     }

//     public PIDController getAngleController(){
//         return anglController;
//     }

//     /**
//      * @return the angleOffset
//      */
//     public double getAngleOffset() 
//     {
//         return angleOffset;
//     }

//     @Override
//     public double getDistance() {
//         return Math.sqrt(Math.pow(getPos() / distancePerPulse * Math.cos(getAngle()), 2) +
//             Math.pow(getPos() / distancePerPulse * Math.sin(getAngle()), 2));
//     }

//     public double getPos(){
//         return driveEncoder.getPosition();
//     }

//     public double getPosInverted(){
//         return getPos() * inverted;
//     }

//     @Override
//     public void setTargetAngle(double angle) {
//         anglController.setSetpoint(angle);
//         this.targetAngle = angle;
//     }

//     public void setTargetDrive(double speed){
//         driveController.setSetpoint(speed);
//         this.targetSpeed = speed;
//     }

//     public double getSetpointAngle(){
//         return Math.toDegrees(anglController.getSetpoint());
//     }

//     @Override
//     public void setDriveOutput(double output) {
//         driveMotor.set(output);
//     }


//     @Override
//     public void updateState(double dt) {
//         super.updateState(dt);
//         angleMotor.set(anglController.calculate(getAngle(), dt));
//         SmartDashboard.putNumber("vel", getAngle());
//     }

//     @Override
//     public void updateStateAuto(double dt){
//         super.updateStateAuto(dt);
//         angleMotor.set(anglController.calculate(getAngle(), dt));
//         SmartDashboard.putNumber("vel", -getVelocity());
//         SmartDashboard.putNumber("motor", driveMotor.getOutputCurrent());
//     }

//     public void DontUseee(double dt){
//         angleMotor.set(anglController.calculate(getAngleDontUse(), dt));
//     }

//     public void resetEncoder(){
//         driveEncoder.setPosition(0);
//     }

//     public void resetEncoderDontUse(){
//         dontUseEncoder.setPosition(0);
//     }
// }
