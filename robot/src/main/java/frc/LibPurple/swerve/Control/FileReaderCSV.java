/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.swerve.Control;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.LineNumberReader;
import java.util.ArrayList;
import java.util.List;

import frc.LibPurple.utils.Utils;
import frc.LibPurple.math.RigidTransform2D;
import frc.LibPurple.math.Rotation2D;
import frc.LibPurple.math.Vector2D;


/**
 * Add your docs here.
 */
public class FileReaderCSV {
	private List<RigidTransform2D> points;
	private List<Rotation2D> rotations;

	private double totalTime = 0;
	private double distance = 0;
	private int direction;
	private int setpointIndex;

	public FileReaderCSV(String fileName)
	{
		this(fileName, false);
	}

	public FileReaderCSV(String fileName, boolean isReversed) 
	{
		if(isReversed) 
			this.direction = -1;
		else 
			this.direction = 1;
		points = new ArrayList<RigidTransform2D>();
		rotations = new ArrayList<Rotation2D>();

		LineNumberReader br = null;

		try {
			br = new LineNumberReader(new FileReader(fileName));
		} catch (FileNotFoundException e1) {
			//TODO log
			e1.printStackTrace();	        
			Utils.printErr(e1.getMessage());
			return;
		}

		try
		{
			br.readLine(); //Ignore first line 'cause Alon said so
			String line = br.readLine();
			String lastLine = "0,";
			int i = 1;

			while (line != null) 
			{
				addLine(line);
				lastLine = line;
				i++;
				br.setLineNumber(i);
				line = br.readLine();
			}
			
			totalTime = Double.parseDouble(lastLine.split(",")[0]);
			
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
		double x = Double.parseDouble(attr[0]); 
		double y = Double.parseDouble(attr[1]); 
		double rotation = Double.parseDouble(attr[2]);
		rotation = Math.toRadians(rotation);
		
		points.add(new RigidTransform2D(new Vector2D(x, y), Rotation2D.fromDegrees(rotation)));
		rotations.add(Rotation2D.fromDegrees(rotation));
	}

	public RigidTransform2D getPoint(int index)
	{
		return points.get(index);
	}

	public int getSizeOfPoints()
	{
		return points.size();
	}

	public Rotation2D getRotation(int index){
		return rotations.get(index);
	}
}