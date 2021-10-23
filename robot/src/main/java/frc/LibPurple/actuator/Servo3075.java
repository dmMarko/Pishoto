package frc.LibPurple.actuator;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Servo3075 extends Servo {

	public Servo3075(int channel) {
		super(channel);
		// TODO Auto-generated constructor stub
	}
	
	/**
	 * Toggle between 2 angles with a joystick button.
	 * Example use: JoystickButton.whenPressed(Servo3075.ButtonToggle(0, 180));
	 * @param angle1 an angle in degrees.
	 * @param angle2 an angle in degrees.
	 * @return a command for the WhenPressed JoystickButton function.
	 */
	public CommandBase ButtonToggle(double angle1, double angle2)
	{
		return new ButtonHandler(this, angle1, angle2);
	}
	
	public CommandBase goToAngle(double angle)
	{
		return new SetAngle(this, angle);
	}
}

class ButtonHandler extends CommandBase
{

	Servo3075 servo;
	double angle1;
	double angle2;
	

	public ButtonHandler(Servo3075 servo, double angle1, double angle2) {
		super();
		this.servo = servo;
		this.angle1 = angle1;
		this.angle2 = angle2;
	}

	@Override
	public void initialize() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void execute() {
		// TODO Auto-generated method stub
		servo.setAngle(angle1 == servo.getAngle() ? angle2 : angle1);
		
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
	}

	@Override
	public void end(boolean interrupted) {
		// TODO Auto-generated method stub
		
	}
}

class SetAngle extends CommandBase
{
	
	Servo3075 servo;
	double angle;
	
	public SetAngle(Servo3075 servo, double angle) {
		// TODO Auto-generated constructor stub
		this.servo = servo;
		this.angle = angle;
	}

	@Override
	public void initialize() {
		// TODO Auto-generated method stub
		servo.setAngle(angle);
	}

	@Override
	public void execute() {
		// TODO Auto-generated method stub
		servo.setAngle(angle);
		
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
	}

	@Override
	public void end(boolean interrupted) {
		// TODO Auto-generated method stub
		
	}
}