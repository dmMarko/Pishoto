/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.sensors;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Add your docs here.
 */
public class AnglePIdSource implements PIDSource
{
    NavX navx;
    AngleType angleType;

    public enum AngleType
    {
        Yaw, Roll, Pitch
    }

    public AnglePIdSource(NavX navx, AngleType angleType)
    {
        this.navx = navx;
        this.angleType = angleType;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        throw new UnsupportedOperationException();
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() 
    {
        if (this.angleType == AngleType.Pitch)
            return this.navx.getPitch();
        else if (this.angleType == AngleType.Roll)
            return this.navx.getRoll();
        else
            return this.navx.getYaw();
	}

}
