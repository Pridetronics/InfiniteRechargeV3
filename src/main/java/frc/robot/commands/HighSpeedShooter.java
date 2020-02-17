/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pneumatics.ballReleasePiston;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class HighSpeedShooter extends CommandBase {
  /**
   * Creates a new LowSpeedShooter.
   */
  private Pneumatics m_pneumatics;
  private Shooter m_shooter; // new shooter variable to store shooter object in
  private Joystick m_joystickShooter; // Joystick variable
  private double m_highShooterSpeed; // allows the low speed constant to be stored in a variable in this command

  public HighSpeedShooter(Joystick joystickShooter , Shooter shooter, Pneumatics pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;// stores shooter object from parameters
    m_joystickShooter = joystickShooter; // stores the joystickShooter object from parameters
    m_pneumatics = pneumatics;
    
    addRequirements(m_shooter);
    addRequirements(m_pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_highShooterSpeed = Constants.highShooterSpeed; // sets the high speed constant to the variable
    m_shooter.shooterSpeed(m_highShooterSpeed); // calls the shooterSpeed method on the shooter object
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_pneumatics.retractGate();
    ReleaseGate.ballRelease = ballReleasePiston.EXTENDED;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
