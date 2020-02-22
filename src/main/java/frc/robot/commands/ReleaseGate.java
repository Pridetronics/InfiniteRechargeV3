/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReleaseGate extends CommandBase {
  /**
   * Creates a new GateRelease.
   */
   private double m_speed;
   private Joystick m_joystickShooter; // empty variables to bring in objects
   private CANEncoder m_shooterMotorEncoder; // creates empty encoder object
   private double m_shooterMotorRPM; // variable represents the rpm of the shooter motor
   private Shooter m_shooter;

  public ReleaseGate(Joystick joystickShooter, Shooter shooter, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_joystickShooter = joystickShooter; // instantiation of empty objects
    //m_pneumatics = pneumatics;
    //ballRelease = ballReleasePiston.EXTENDED; // sets the default state of the piston to extended
    m_shooterMotorEncoder = RobotContainer.shooterMotorEncoder; // references an encoder object
    m_speed = speed; // speed depending on which shooter is used(low or high)
    m_shooterMotorRPM = Constants.lowShooterSpeed; // represents shooter RPM
    m_shooter = shooter;

    //addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    
    /*
      if statement to test whether the motor has gotten up to speed. When it is up to speed, it runs what is
      inside the if statement.
    */
    /*
    if(m_shooterMotorEncoder.getVelocity() == m_shooterMotorRPM * m_speed)
    {
      if(ballRelease == ballReleasePiston.EXTENDED) // if the ballReleasePiston is extended, release the gate
      {                                             // and change the state of the piston
        m_pneumatics.releaseGate();
        ballRelease = ballReleasePiston.RETRACTED;
      }
      else                                          // if the ballReleasePiston is retract, close the gate
      {                                             // and change the state of the piston
       m_pneumatics.retractGate();
        ballRelease = ballReleasePiston.EXTENDED;
      }
    }
    */
    
    
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_shooter.releaseGate();
    //ballRelease = ballReleasePiston.RETRACTED;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean commandState = false;
    if(m_shooterMotorEncoder.getVelocity() == 5676 * m_speed)
    {
      commandState = true;
    }
    return commandState;
  }
}
