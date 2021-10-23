/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.utils;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;


/**
 * Add your docs here.
 */
public class AdbBridge 
{
	Path binLocation;
	public final static Path DEFAULT_PATH = Paths.get("/usr/bin/adb");
	
	public AdbBridge()
	{
		binLocation = DEFAULT_PATH;
	}
	
	public AdbBridge(Path location)
	{
		binLocation = location;
	}
	
	public boolean runCommand(String args)
	{
		// Runtime r = Runtime.getRuntime();
		String cmd = binLocation.toString() + " " + args;

		return runShellCommand(cmd);
	}

	public boolean runShellCommand(String cmd)
	{
		cmd = "sudo " + cmd; 
		Runtime r = Runtime.getRuntime();
		// Utils.print(cmd);
		try{
			Process p = r.exec(cmd);
			p.waitFor();
			// Scanner in = new Scanner(p.getInputStream());
			// Utils.print(in.next());
			
		}catch (IOException e)
		{
			System.err.println("AdbBridge: could not run command " + cmd);
			e.printStackTrace();
			return false;
		}catch (InterruptedException e)
		{
			System.err.println("AdbBridge: could not run command " + cmd);
			e.printStackTrace();
			return false;
		}
		return true;
	}
	
	public void startAdb()
	{
		runCommand("start-server");
	}
	
	public void stopAdb()
	{
		runCommand("kill-server");
	}
	
	public void restartAdb()
	{
		stopAdb();
		startAdb();
	}
	
	public void reversePortForward(int remotePort, int localPort)
	{
		runCommand("reverse tcp:" + remotePort + " tcp:" + localPort);
	}
	
	public void openApp()
	{
		runCommand("shell am start -n com.example.guy.a3075visionproctest/com.example.guy.a3075visionproctest.MainActivity");
	}
}

