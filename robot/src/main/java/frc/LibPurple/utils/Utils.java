package frc.LibPurple.utils;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.BasicFileAttributes;
import java.nio.file.attribute.FileTime;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.Command;

public class Utils
{

	// Takes two numbers and check if they are closer than dif
	public static boolean inRange(double value, double target, double deviation)
	{
		return Math.abs(value-target) <= deviation;
	}

	public static double deadband(double value, double deadband)
	{
		return Math.abs(value) < deadband ? 0 : Math.signum(value) * (Math.abs(value) - deadband) / (1 - deadband);
	}

	public static double accellimit(double currentValue, double lastValue, double accellimit)
	{
		//		double acc = Math.signum(currentValue - lastValue) > 0 ? accellimit : deaccelimit;
		return  lastValue + (Math.signum(currentValue - lastValue) * Math.min(accellimit, Math.abs(currentValue - lastValue)));
	}

	public static double motorBound(double value, double motorBound)
	{
		return Math.signum(value) * (Math.abs(value) - (motorBound * Math.abs(value)) + motorBound);
	}

	public static double[] arcadeDrive(double y, double x)
	{
		double[] arr = new double[2];
		double max = Math.max(Math.abs(y), Math.abs(x));
		double sum = y + x;
		double dif = y - x;
		if(y >= 0)
		{
			if(x >= 0)
			{
				arr[0] = max;
				arr[1] = dif;
			}
			else
			{
				arr[0] = sum;
				arr[1] = max;
			}
		}
		else
		{
			if(x >= 0)
			{
				arr[0] = sum;
				arr[1] = -max;
			}
			else
			{
				arr[0] = -max;
				arr[1] = dif;
			}
		}
		max = 0;
		sum = 0;
		dif = 0;
		return arr;
	}

	public static double[] arcadeDrive(double y, double x, double turnSensitivity)
	{
		return arcadeDrive(y, x * turnSensitivity);
	}

	public static double powerValue(double value, double power)
	{
		return Math.signum(value)*Math.abs(Math.pow(Math.abs(value), power));
	}

	/**
	 * Prints an error to DriverStation with the cause and location
	 * @param err The error message
	 */
	public static void printErr(String err)
	{
		DriverStation.reportError(err, true);
	}

	/**
	 * Prints a message to DriverStation
	 * @param msg The message you want to print
	 */
	public static void print(String msg)
	{
		DriverStation.reportError(msg, false);;
	}

	// public static void batteryWatcher()
	// {
	// 	if(DriverStation.getInstance().getBatteryVoltage() < 10)
	// 		print("Battery is low! (Battery below 10V)");
	// }

	public static void clearSticky()
	{
		PowerDistributionPanel pdp = new PowerDistributionPanel();
		pdp.clearStickyFaults();
	}

	public static double[] driveCurve(double radius, double angle, double robotWidth)
	{
		double[] arr = {(radius-robotWidth/2)*angle, (radius+robotWidth/2)*angle};
		return arr;
	}

	/**
	 * This is a requires like the Command's one but for any object.
	 * The given Command will be canceled if the given object is null.
	 * Use inside a command example: requires(this, Components.systemStick);
	 * @param command the command that should be canceld of the object is null.
	 * @param object the object that is required.
	 * Contact Shahar for help.
	 */
	public static void requires(Command command, Object object)
	{
		if(object == null)
			command.cancel();
	}

	/**
	 * Calculates the y value of a point on a linear interpolation function.
	 * @param points A sorted Point3075 array that will be used for the intepolated function
	 * @param x The value for calculating its y.
	 * @return y value for the specified x.
	 */
	public static double linearInterpolation(Point3075[] points, double x)
	{
		// Making sure that x is within the points array range.
		// if not, return its y value on the closest line.
		if (x < points[0].getX())
		{
			return Point3075.getLine(points[0], points[1], x);
		}
		else if (x > points[points.length - 1].getX())
		{
			return Point3075.getLine(points[points.length - 1], points[points.length - 2], x);
		}

		for(int i = 0; i < points.length - 1; i++)
		{
			if(points[i].getX() < x && points[i + 1].getX() > x)
			{
				return Point3075.getLine(points[i], points[i + 1], x);
			}
		}

		// Prevent return errors, this should never happen
		return 0;
	}


	public static FileWriter initializeCSVFile(String name , int maxFilesInFolder)
	{
		DateFormat dd = new SimpleDateFormat("hh:mm:ss");
		Date time = new Date();
		try 
		{
			deleteOldFiles(maxFilesInFolder-1, name + "_" + dd.format(time) + ".csv");
			return new FileWriter(name + "_" + dd.format(time) + ".csv");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
//			Utils.print(e.getMessage());
		}
		return null;
	}	
	
	public static FileWriter initializeCSVFile(String name)
	{
		DateFormat dd = new SimpleDateFormat("hh:mm:ss");
		Date time = new Date();
		try 
		{
			return new FileWriter(name + "_" + dd.format(time) + ".csv");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
//			Utils.print(e.getMessage());
		}
		return null;
	}

	public static void deleteOldFiles(int maxFiles , String fileName)
	{
		String path = "";
		String run = "";
		for(int i = 0; i < fileName.length(); i++)
		{
			run += fileName.charAt(i);
			if (fileName.charAt(i) == '/')
			{
				path += run;
			}
		}
		final File folder = new File(path);
		ArrayList<String> filesNames = listFilesForFolder(folder);
		ArrayList<Long> time = new ArrayList<Long>();
		if (filesNames.size() > maxFiles-1)
		{
			for(int i = 0;i < filesNames.size(); i++)
			{
				time.add(creationTime(path, filesNames.get(i)).to(TimeUnit.SECONDS));
			}
		}
		time.sort(Collections.reverseOrder());
		for(int i = 0; i < time.size(); i++)
		{
			long timeFile = creationTime(path, filesNames.get(i)).to(TimeUnit.SECONDS);
			for(int j = maxFiles; j < filesNames.size(); j++)
			{
				if(time.get(j) == timeFile)
					try {
						Files.delete(Paths.get(path, filesNames.get(i)));
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
			}
		}
	}
	public static FileTime creationTime(String folderPath , String fileName) {
		Path path = Paths.get(folderPath, fileName);	
		BasicFileAttributes attributes = null;
		try {
			attributes = Files.readAttributes(path, BasicFileAttributes.class);
		} catch (IOException e) {
			e.printStackTrace();
		}
		return attributes.creationTime();
	}

	public static ArrayList<String> listFilesForFolder(final File folder) {
		ArrayList<String> filesNames = new ArrayList<String>();
		for (final File fileEntry : folder.listFiles()) {
			if (fileEntry.isDirectory()) {
				listFilesForFolder(fileEntry);
			} else {
				filesNames.add(fileEntry.getName());
			}
		}
		return filesNames;
	}
	

	
	public static void addCSVLine(FileWriter writer, double[] params)
	{

		try
		{
			int len = params.length;
			for(int i = 0; i < len - 1; i++)
			{
				writer.write(Double.toString(params[i]));
				writer.write("\t,");
			}
		
			writer.write(Double.toString(params[len - 1]));
			writer.write('\n');

		}
		catch (IOException e) 
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
	
	public static void addCSVLine(FileWriter writer, String[] params)
	{

		try
		{
			int len = params.length;
			for(int i = 0; i < len - 1; i++)
			{
				writer.write(params[i]);
				writer.write("\t,");
			}
		
			writer.write(params[len - 1]);
			writer.write('\n');

		}
		catch (IOException e) 
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	public static void closeCSVFile(FileWriter writer)
	{
		try {
			writer.flush();
			writer.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// In the memory of our beloved Kellner

		//	public static double[] driveCurveByDistance(double radius, double distance, double robotWidth)
		//	{
		//		double[] arr = {(radius-robotWidth/2)*distance/radius, (radius+robotWidth/2)*distance/radius};
		//		return arr;
		//	}

		//	public static double[] driveCurveByAngle(double angle, double distance, double robotWidth)
		//	{
		//		double[] arr = {distance - (robotWidth/2)*angle, distance + (robotWidth/2)*angle};
		//		return arr;
		//	}
	}
}