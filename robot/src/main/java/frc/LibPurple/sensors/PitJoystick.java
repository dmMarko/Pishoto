package frc.LibPurple.sensors;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.hal.HAL;

public class PitJoystick extends GenericHID {

    /**
     * Construct an instance of a joystick. The joystick index is the USB port on the dirvers station. 
     * 
     * @param port The port on the Driver station that the joystick is plugged into.
     */
    public PitJoystick(int port) {
        super(port);
        
        HAL.report(tResourceType.kResourceType_Joystick, port + 1);
    }

    @Override
    public double getX(Hand hand) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getY(Hand hand) {
        // TODO Auto-generated method stub
        return 0;
    }
    
}
