package frc.LibPurple.Tank.trajectories;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;


import frc.LibPurple.utils.Utils;


/* CSV File format
 * time, x, y, θ
 * 1,5,6,13
 * 2,5,7,14
 */

public class TrajectoryFile implements Trajectory 
{

	private List<TimeStamp> stamps;

	private double totalTime = 0;
	private double distance = 0;
	private int direction;
	private int setpointIndex;

	public TrajectoryFile(String fileName)
	{
		this(fileName, false);
	}

	public TrajectoryFile(String fileName, boolean isReversed) 
	{
		if(isReversed) 
			this.direction = -1;
		else 
			this.direction = 1;
		stamps = new ArrayList<TimeStamp>();

		BufferedReader br = null;

		try {
			br = new BufferedReader(new FileReader(fileName));
		} catch (FileNotFoundException e1) {
			e1.printStackTrace();	        
			Utils.printErr(e1.getMessage());
			return;
		}

		try
		{
			br.readLine(); //Ignore first line 'cause Alon said so
			String line = br.readLine();
			String lastLine = "0,";

			while (line != null) 
			{
				addLine(line);
				lastLine = line;
				line = br.readLine();
			}

			totalTime = Double.parseDouble(lastLine.split(",")[0]);
			// distance = Double.parseDouble(lastLine.split(",")[1]);

		} catch(Exception e) {
			try {
				br.close();
			} catch (IOException e1) {
				Utils.printErr("[Trajectory File]: Error closing file");
			}
		}

		try {
			br.close();
		} catch (IOException e) {
			Utils.printErr("[Trajectory File]: Error closing file");
		}
	}

	private void addLine(String line) 
	{
		String[] attr = line.split(",");

		double time = Double.parseDouble(attr[0]);
		double x = Double.parseDouble(attr[1]) * this.direction;
		double y = Double.parseDouble(attr[2]) * this.direction;
		double angle = Double.parseDouble(attr[3]);

		stamps.add(new TimeStamp(time, x, y, angle));
	}

	@Override
	public double[] calculate(double time)
	{
		try {
			TimeStamp currStamp = null; //why 2 lines?

			if(time >= totalTime)
			{
				currStamp = stamps.get(stamps.size()-1);
			}
			else
			{
				setpointIndex = (int) ((time / totalTime) * stamps.size());
				currStamp = stamps.get(setpointIndex);
			}
			double[] setpoint = new double[3];
			setpoint[0] = currStamp.x; //* direction;
			setpoint[1] = currStamp.y; //* direction;
			setpoint[2] = currStamp.angle; //* direction;

			return setpoint;

		} catch (Exception e) {
			return null;
		}
	}

	@Override
	public double getTotalTime() 
	{
		return totalTime;
	}

	
	// public double getDistance()
	// {
	// 	return this.dstance * direction;
	// }

	@Override
	public int getDirection() {
		return this.direction;
	}

	@Override
	public double[] getNextSetpoint() 
	{
		try {
			TimeStamp currStamp = null; //why 2 lines?

			if(setpointIndex >= stamps.size() - 1)
			{
				currStamp = stamps.get(stamps.size()-1);
			}
			else
			{
				currStamp = stamps.get(setpointIndex + 1);
			}
			double[] setpoint = new double[3];
			setpoint[0] = currStamp.x;
			setpoint[1] = currStamp.y;
			setpoint[2] = currStamp.angle;

			return setpoint;

		} catch (Exception e) {
			return null;
		}
	}
}
