/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class LowSpeedShooter extends CommandBase 
{
  /**
   * Creates a new LowSpeedShooter.
   */
  private Shooter m_shooter; // new shooter variable to store shooter object in

  public LowSpeedShooter(Shooter shooter) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter; // stores shooter object from parameters
    
    addRequirements(m_shooter); // Declares the dependencies of the command
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
    m_shooter.shooterOn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_shooter.shooterOff();
    m_shooter.retractGate(); // will bring the gate back up after the shooting has finished
    //ReleaseGate.ballRelease = ballReleasePiston.EXTENDED;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
