/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.LibPurple.sensors.ConsoleJoystick;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Elevator.moveElevatorManual;
import frc.robot.commands.Elevator.setElevatorPosition;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.arm.SetArmPos;
import frc.robot.commands.intake.CollectDiscFromFloor;
import frc.robot.commands.intake.CollectDiskFromFloor;
import frc.robot.commands.intake.OutTakeDisc;
import frc.robot.commands.intake.TakeDisc;
import frc.robot.commands.intake.Helpers.setPiston;
import frc.robot.commands.intake.Helpers.setPiston.Piston;
// import frc.robot.commands.intake.CollectDiskFromFloor;
// import frc.robot.commands.intake.TakeDisc;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Movement;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static final Intake intake = new Intake();
  public static final Elevator elevator = new Elevator();
  public static final Arm arm = new Arm();
  // public static final Movement drive = new Movement();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  
  public static final ConsoleJoystick driver = new ConsoleJoystick(0);
  public static final ConsoleJoystick commander = new ConsoleJoystick(1);

  public static JoystickButton a = new JoystickButton(driver, 1);
  public static JoystickButton b = new JoystickButton(driver, 2);
  public static JoystickButton x = new JoystickButton(driver, 3);
  public static JoystickButton y = new JoystickButton(driver, 4);
  public static POVButton up = new POVButton(driver, 0);
  public static POVButton down = new POVButton(driver, 180);
  public static POVButton right = new POVButton(driver, 90);
  public static POVButton left = new POVButton(driver, 270);

  private int pos = 0;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    a.toggleWhenPressed(new setPiston(false, Piston.Floor));
    b.toggleWhenPressed(new CollectDiskFromFloor());
    x.toggleWhenPressed(new OutTakeDisc(20000));
    y.toggleWhenPressed(new TakeDisc());

    up.toggleWhenPressed(new SetArmAngle(60));
    down.toggleWhenPressed(new SetArmAngle(30));
    right.toggleWhenPressed(new OutTakeDisc(4000));
    left.toggleWhenPressed(new moveElevatorManual());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
