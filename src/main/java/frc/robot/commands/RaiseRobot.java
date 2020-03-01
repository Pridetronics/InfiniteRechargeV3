/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class RaiseRobot extends CommandBase 
{
  /**
   * Creates a new RaiseRobot.
   */
  //Creates a climb object
  private Climb m_climb;
  
  public RaiseRobot(Climb climb) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    //References climb object from robot container
    m_climb = climb;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //Runs the winch motor to spool the winch
    m_climb.spoolWinch(Constants.WINCH_MOTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    //This may not be a good idea
    //m_climb.spoolWinch(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
