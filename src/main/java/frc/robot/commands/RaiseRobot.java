/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class RaiseRobot extends TimedCommand 
{
  /**
   * Creates a new RaiseRobot.
   */
  private Climb m_climb;
  
  
  public RaiseRobot(double timeout, Climb climb) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    super(timeout);
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
    m_climb.spoolWinch(Constants.WINCH_MOTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() 
  {
    m_climb.spoolWinch(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}