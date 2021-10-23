package frc.LibPurple.actuator;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class Solenoid3075 extends DoubleSolenoid{

	public Solenoid3075(int forwardChannel, int reverseChannel) {
		super(forwardChannel, reverseChannel);
		// TODO Auto-generated constructor stub
	}
	
	public Solenoid3075(int moduleNum, int forwardChannel, int reverseChannel) {
		super(moduleNum, forwardChannel, reverseChannel);
		// TODO Auto-generated constructor stub
	}
	
	public CommandBase ToggleCommand()
	{
		return new Toggle(this);
	}
	
	public CommandBase OpenCommand()
	{
		return new OpenClose(this, true);
	}
	
	public CommandBase CloseCommand()
	{
		return new OpenClose(this, false);
	}
	
	public CommandBase OffCommand()
	{
		return new Off(this);
	}
	
	public CommandBase TimedCycle(double timeOut)
	{
		return new TimedCycle(this, timeOut);
	}
}



class Toggle extends CommandBase {

	DoubleSolenoid mySol;
	
	public Toggle(DoubleSolenoid ds)
	{
		mySol = ds;
	}

    public void initialize() {
    }

    public void execute() {
    	mySol.set(mySol.get() == DoubleSolenoid.Value.kForward ? 
    			DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {
    }
}

class OpenClose extends CommandBase {

	DoubleSolenoid mySol;
	boolean open;
	
    public OpenClose(DoubleSolenoid ds, boolean open) {
    	mySol = ds;
    	this.open = open;
    }

    public void initialize() {
    }

    public void execute() {
    	mySol.set(open ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {
    }
}

class TimedCycle extends CommandBase {

	DoubleSolenoid mySol;
	
    public TimedCycle(DoubleSolenoid ds, double timeOut) {
    	mySol = ds;
    	withTimeout(timeOut);
    }

    public void initialize() {
    	mySol.set(DoubleSolenoid.Value.kForward);
    }

    public void execute() {
    }

    public boolean isFinished() {
        // return isTimedOut();
        return false;
    }

    public void end(boolean interrupted) {
    	mySol.set(DoubleSolenoid.Value.kReverse);
    }
}

class Off extends CommandBase{
DoubleSolenoid mySol;
	
    public Off(DoubleSolenoid ds) {
    	mySol = ds;
    }

    public void initialize() {
    	mySol.set(DoubleSolenoid.Value.kOff);
    }

    public void execute() {
    	
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {
    	
    }
}