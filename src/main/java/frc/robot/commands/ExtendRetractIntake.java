/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ExtendRetractIntake extends CommandBase 
{
  /**
   * Creates a new ExtendRetractIntake.
   */
  //Creates intake object
  private Intake m_intake;
  //boolean isIntakeExtended = false;

  public ExtendRetractIntake(Intake intake) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    //References intake object in RobotContainer
    m_intake = intake;

    addRequirements(m_intake);
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
    //Extends intake when the command is executed
    m_intake.extendIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    //Retracts the intake when the command ends
    m_intake.retractIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
