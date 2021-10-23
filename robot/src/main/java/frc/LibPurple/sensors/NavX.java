package frc.LibPurple.sensors;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.RobotMap;

public class NavX {

    private AHRS navX;
    private double yawAng = 0;
    private double pitchAng = 0;
    private double rollAng = 0;

  public NavX()
  {
     this.navX = new AHRS(SPI.Port.kMXP, (byte) 200);
    //  this.navX = new AHRS(RobotMap.navXPort, (byte) 200);
  }

  public double getYaw()
  {
    return navX.getYaw();
  }

  public double getYawRad()
  {
    return Math.toRadians(this.navX.getYaw());
  }

  public double getRoll()
  {
    return navX.getRoll();
  }

  public double getPitch()
  {
    return navX.getPitch();
  } 

  public double getXAngularVelocity()
  {
    return navX.getRawGyroX();
  }

  public double getYAngularVelocity()
  {
    return navX.getRawGyroY();
  }

  public double getZAngularVelocity()
  {
    return navX.getRawGyroZ();
  }

  public double getAccelerationX()
  {
    return navX.getRawAccelX();
  }

  public double getAccelerationY()
  {
    return navX.getRawAccelY();
  }

  public double getAccelerationZ()
  {
    return navX.getRawAccelZ();
  }

  public void resetYaw()
  {
    navX.zeroYaw();
  }

  public double getMagnetomerYaw()
  {
    return this.navX.getCompassHeading();
  }

  public double getFusedYaw()
  {
    return this.navX.getFusedHeading();
  }

  public double getUpdateRate()
  {
    return this.navX.getActualUpdateRate();
  }

  public double getDisplacementX()
  {
    return this.navX.getDisplacementX();
  }

  public double getDisplacementY()
  {
    return this.navX.getDisplacementY();
  }

  public void reset()
  {
    this.navX.resetDisplacement();
    this.navX.reset();
  }

}