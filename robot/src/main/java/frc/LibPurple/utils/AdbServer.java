/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.utils;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Scanner;
import java.util.concurrent.BlockingQueue;





/**
 * Add your docs here.
 */
public class AdbServer extends Thread
{
	// public String str;
	public BlockingQueue<String> queue;
	private boolean isAlive;
	public String str = "";
	public Point3075 center;
	private static ServerSocket ss;
	private Socket conn;
	private Scanner scanner;
	
	public AdbServer(BlockingQueue<String> queue)
	{
		this.queue = queue;
		this.isAlive = true;
//		thread = new Thread(this);
	}

	public void run()
	{
		this.isAlive = true;

		try{
			connect(3076);
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
		while (this.isAlive)
		{
			// try {
				// Utils.print(this.queue.size() + "");
				// Utils.print("xx");
				str = getString();
				center = getCenter();
				// Robot.adb.r                                                                                                                                                      unCommand("shell input tap 1 101");
				// this.queue.put(getString());
			// } catch (InterruptedException e) {
				// e.printStackTrace();
			// }
		}

	}
	public void killThisFucker()
	{
		this.isAlive = false;
	}

	public void connect(int port) throws IOException
	{	
		Utils.print("1\n");

		if(ss == null)
			ss = new ServerSocket(port);
		else
		{
			ss.close();
			ss = new ServerSocket(port);
		}

		Utils.print("2\n");
		// t.start();
		conn = ss.accept();
		Utils.print("Mazal tov\n");
		Scanner s = new Scanner(conn.getInputStream());
		scanner = s;
	}

	public boolean isConnceted()
	{
		return ss != null && conn != null && scanner != null;
	}
	
	public String getString()
	{
		if(ss != null && conn != null && scanner != null)
		{
			try
			{
				// Ut`ils.print(scanner.nextLine());
				conn.getInputStream().skip(conn.getInputStream().available());
			}
			catch (IOException e)
			{
				e.printStackTrace();
			}
			try
			{
				// Utils.print("msg");
				// if(br.ready())
				// {
				return scanner.nextLine();
					// return br.readLine();
				// }
				// else
				// 	return "700 360";

			}
			catch(Exception e)
			{
				// e.printStackTrace();
				// Utils.print("DIE!!!");
				return "700 360";
			}
		}
		else
		{
			Utils.print("avi");
			return null;
		}
	}

	public Double getDouble()
	{
		try
		{
			conn.getInputStream().skip(conn.getInputStream().available());
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
		return scanner.nextDouble();
	}

	public Point3075 getCenter()
	{
		Point3075 p = new Point3075();

		String line = str;

		if(line != null)
		{
			p.x = Double.parseDouble(line.split(" ")[0]);
			p.y = Double.parseDouble(line.split(" ")[1]);
		}
		else
		{
			p.x = -2;
			p.y = -2;
		}

		return p;
	}

	public void close()
	{
		if(ss != null && scanner != null && conn != null)
		{
			try
			{
				ss.close();
				scanner.close();
				conn.close();
			}
			catch (IOException e)
			{
				e.printStackTrace();
			}
		}
	}
}
