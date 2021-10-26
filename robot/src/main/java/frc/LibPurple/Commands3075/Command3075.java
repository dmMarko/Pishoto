/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.Commands3075;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Add your docs here.
 */
public class Command3075 {

    private Command command;
    private Enum entry;

    public Command3075(Command command, Enum entry){
        this.command = command;
        this.entry = entry;
    }

    public Command getCommand(){
        return this.command;
    }

    public Enum getEntry(){
        return this.entry;
    }

    
}
