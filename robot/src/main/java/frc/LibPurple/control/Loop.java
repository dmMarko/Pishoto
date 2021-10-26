package frc.LibPurple.control;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.RobotController;
import frc.LibPurple.swerve.Utils.Utils;
import frc.robot.RobotContainer;



public abstract class Loop implements Runnable
{
    protected ScheduledExecutorService scheduler;
    protected ScheduledFuture sch;
    protected boolean enabled = false;

    protected double period; 

    public Loop()
	{
        period = 0.01;
		this.scheduler = Executors.newSingleThreadScheduledExecutor();
    }

    @Override
    public void run() 
    {
        if (!enabled) 
        {
            disable();
			return;
        }
    }

    private void stop()
    {
        this.sch.cancel(true);
    }
    
    /**
     * starts loop and disable run
     */
    private void start()
    {
        sch = scheduler.scheduleAtFixedRate(this, 0L, (long)(this.period * 1000), TimeUnit.MILLISECONDS);
    }

    public void disable()
	{
        this.enabled = false;
        stop();
    }
    
    public void enable()
    {
        start();
        this.enabled = true;
    }
}
