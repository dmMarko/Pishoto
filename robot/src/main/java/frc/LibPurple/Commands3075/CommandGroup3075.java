/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.Commands3075;

import java.util.ArrayList;
import java.util.List;

import javax.print.DocFlavor.READER;

import edu.wpi.first.wpilibj2.command.Command;

/**
* Add your docs here.
*/
public class CommandGroup3075 extends CommandGroupBase3075 {
  
  public enum Entry {
    parallel, sequential
  }
  
  private final List<Command3075> m_commands = new ArrayList<>();
  private int m_currentCommandIndex = -1;
  private boolean m_runWhenDisabled = true;
  
  /**
  * Creates a new SequentialCommandGroup. The given commands will be run
  * sequentially, with the CommandGroup finishing when the last command finishes.
  *
  * @param commands the commands to include in this group.
  */




  /**
   * TO DO:
   *  - add exceptions
   *  - check requirements
   */
  public CommandGroup3075(Command... commands) {
  }
  
  @Override
  public void addParallel(Command command) {
    // TODO Auto-generated method stub
    m_commands.add(new Command3075(command, Entry.parallel));
    
  }
  
  @Override
  public void addSequential(Command command) {
    // TODO Auto-generated method stub
    m_commands.add(new Command3075(command, Entry.sequential));
    
  }
  
  @Override
  public void initialize() {
    
    
  }
  
  @Override
  public void execute() {
    for (int i = 0; i < m_commands.size(); i++) {
      if(isSequential(m_commands.get(i))){
        m_commands.get(i).getCommand().initialize();
        if(isNextParallel(m_commands.get(i))){
          runParallelSequence(i+1);  //need to fix this in the runParallelSeq func 
        }
      }else{
        runParallelSequence(i);
      }
    }
    
  }
  
  @Override
  public void end(boolean interrupted) {
    
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
  
  @Override
  public boolean runsWhenDisabled() {
    return m_runWhenDisabled;
  }
  
  private Command3075 getNextCommand(Command command) {
    
    for (int i = 0; i < m_commands.size() - 1; i++) {
      if (m_commands.get(i).equals(command)) {
        return m_commands.get(i + 1);
      }
    }
    return null;
  }
  
  private boolean isNextParallel(Command3075 command) {
    return isParallel(getNextCommand(command.getCommand()));
  }
  
  private int runParallelSequence(int startIndex) {
    int count = 0;
    for (int i = startIndex; i < m_commands.size(); i++) {
      if (isParallel(m_commands.get(i))) {
        m_commands.get(i).getCommand().initialize();
        count++;
      }
    }
    return count;
  }
  
  private boolean isCurrentCommandRunning(Command3075 command){
    return command.getCommand().isFinished();
  }
  
  private boolean isSequential(Command3075 command){
    return command.getEntry().equals("Sequential");
  }
  private boolean isParallel(Command3075 command){
    return command.getEntry().equals("Parallel");

  }
  //not so sure on this one 
  // should run all the parallel run time 
  // returns true if all parallel commands ended 

  private boolean isParallelSequenceFinished(int startIndex, int endIndex){
    boolean isFinishedState = false;
    while(startIndex <= endIndex){
      isFinishedState = m_commands.get(startIndex).getCommand().isFinished();
      startIndex++;
    }
    return isFinishedState;
  }
  
}
