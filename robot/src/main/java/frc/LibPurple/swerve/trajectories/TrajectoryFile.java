package frc.LibPurple.swerve.trajectories;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import frc.LibPurple.math.Rotation2D;
import frc.LibPurple.math.Vector2D;
import frc.LibPurple.utils.Utils;



/* CSV File format
 * time, x, y, rotation
 * 1,5,6,13
 * 2,5,7,14
 */

public class TrajectoryFile implements Trajectory
{

    private List<TimeStamp> stamps;
    private List<Segment> segments;

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
        segments = new ArrayList<Segment>();

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
            addAcceleration();

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
		double x = Double.parseDouble(attr[1]);
		double y = Double.parseDouble(attr[2]);
        double heading = Double.parseDouble(attr[3]);
        double velocity = Double.parseDouble(attr[4]);
		double rotation = Double.parseDouble(attr[5]);
		if (direction == -1){
			rotation += 180;
		}
        

	
        stamps.add(new TimeStamp(time, new Vector2D(x, y), Rotation2D.fromDegrees(heading), Rotation2D.fromDegrees(rotation), velocity));
    }
    
    private void addAcceleration(){
        TimeStamp currTimeStamp = stamps.get(0);
        segments.add(new Segment(currTimeStamp.translation, currTimeStamp.heading, currTimeStamp.rotation, currTimeStamp.velocity, 0));
        for(int i = 1; i < stamps.size() - 1; i++){
            currTimeStamp = stamps.get(i);
            double acceleration = (stamps.get(i + 1).velocity - stamps.get(i - 1).velocity) / (stamps.get(i + 1).time - stamps.get(i - 1).time);
            segments.add(new Segment(currTimeStamp.translation, currTimeStamp.heading, currTimeStamp.rotation, currTimeStamp.velocity, acceleration));
        }
        currTimeStamp = stamps.get(stamps.size() - 1);
        segments.add(new Segment(currTimeStamp.translation, currTimeStamp.heading, currTimeStamp.rotation, currTimeStamp.velocity, 0));
    }
    
    @Override
    public Segment calculate(double time){
        try{
            Segment currSeg = null;

            if(time >= totalTime){
                currSeg = segments.get(segments.size() - 1);
            }
            else{
                setpointIndex = (int) ((time / totalTime) * segments.size());
                currSeg = segments.get(setpointIndex);
			}
			
            return new Segment(currSeg.translation, currSeg.heading, currSeg.rotation, currSeg.velocity, currSeg.acceleration);
        } catch (Exception e){
            return null;
        }
    }

	public double getTotalTime() 
	{
		return totalTime;
	}

	
	// public double getDistance()
	// {
	// 	return this.dstance * direction;
	// }

	public int getDirection() {
		return this.direction;
	}
    
    public Segment getNextSetpoint() 
	{
		try {
			Segment currSeg = null; //why 2 lines?

			if(setpointIndex >= segments.size() - 1)
			{
				currSeg = segments.get(segments.size()-1);
			}
			else
			{
				currSeg = segments.get(setpointIndex + 1);
            }
            
            return new Segment(currSeg.translation, currSeg.heading, currSeg.rotation, currSeg.velocity, currSeg.acceleration);

		} catch (Exception e) {
			return null;
		}
	}

	@Override
	public Segment getLastSegment(){
		return segments.get(segments.size() - 1);
	}
}
