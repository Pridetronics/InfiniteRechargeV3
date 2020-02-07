/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pneumatics.ballReleasePiston;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReleaseGate extends CommandBase {
  /**
   * Creates a new GateRelease.
   */

   private Joystick m_joystickShooter;
   private Pneumatics m_pneumatics;
   public static ballReleasePiston ballRelease;
   private CANEncoder m_shooterMotorEncoder;
   boolean boolin = true;

  public ReleaseGate(Joystick joystickShooter, Pneumatics pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_joystickShooter = joystickShooter;
    m_pneumatics = pneumatics;
    ballRelease = ballReleasePiston.EXTENDED;
    m_shooterMotorEncoder = RobotContainer.shooterMotorEncoder;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_pneumatics.releaseGate();
    //m_pneumatics.gateControl(ballRelease);
    
    while(!boolin)
    {
      if(m_shooterMotorEncoder.getVelocity() == 5675 * 0.8)
      {
        boolin = false;
      }
    }
    
    if(ballRelease == ballReleasePiston.EXTENDED)
    {
      m_pneumatics.releaseGate();
      ballRelease = ballReleasePiston.RETRACTED;
      /*
        Note to self(will be deleted later):
        This command could be ran in parallel with the shooter commands. To do that, a loop could be
        used in execute to ensure the motor is up to speed. A more advanced way may be hidden in 
        the schedule command class.
      */
    }
    else
    {
      m_pneumatics.retractGate();
      ballRelease = ballReleasePiston.EXTENDED;
    }
  
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
