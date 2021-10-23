package frc.LibPurple.utils;

import java.util.Arrays;
import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class MedianFilter implements PIDSource{

	private PriorityQueue<Double> inputs;
	private AnalogPotentiometer pot;
	
	public MedianFilter(AnalogPotentiometer pot)
	{
		this.pot = pot;
		this.inputs = new PriorityQueue<>();
		inputs.add(0.0);
		inputs.add(0.0);
		inputs.add(0.0);
//		inputs.add(0.0);
//		inputs.add(0.0);
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override	
	public double pidGet() {
		inputs.add(this.pot.get());
		inputs.remove();
		Double[] ins = new Double[3];
		inputs.toArray(ins);
		Arrays.sort(ins);
		return ins[1];
	}
	
	public String getQueue()
	{
		return this.inputs.toString();
	}
}
