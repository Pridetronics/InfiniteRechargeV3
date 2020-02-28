/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANEncoder;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReleaseGate extends CommandBase 
{
  /**
   * Creates a new GateRelease.
   */
   private CANEncoder m_shooterMotorEncoder; // creates empty encoder object
   private Shooter m_shooter; // creates shooter object

  public ReleaseGate(Shooter shooter) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterMotorEncoder = RobotContainer.shooterMotorEncoder; // references an encoder object
    m_shooter = shooter; // references shooter object in robot container

    //addRequirements(m_shooter);
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
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_shooter.releaseGate(); // when it ends, the gate is released, letting the balls fall through
    //ballRelease = ballReleasePiston.RETRACTED;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    //Tests that the shooter motor is up to desired RPM, and ends the command
    boolean commandState = false;
    if(m_shooterMotorEncoder.getVelocity() == Constants.SHOOTER_DESIRED_RPM)
    {
      commandState = true;
    }
    return commandState;
  }
}
