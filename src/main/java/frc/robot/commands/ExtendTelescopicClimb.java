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

public class ExtendTelescopicClimb extends CommandBase
{
  /**
   * Creates a new ExtendTelescopicClimb.
   */
  private Climb m_climb;

  public ExtendTelescopicClimb(Climb climb) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climb = climb;

    addRequirements(m_climb);
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
    if(m_climb.isClimbAtTop() == false)
    {
      m_climb.raiseTelescopicRod(Constants.TELESCOPIC_ROD_MOTOR_SPEED);
    }
    else
    {
      m_climb.raiseTelescopicRod(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_climb.raiseTelescopicRod(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
