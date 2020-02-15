/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class LowSpeedShooter extends CommandBase {
  /**
   * Creates a new LowSpeedShooter.
   */
  private Shooter m_shooter; // new shooter variable to store shooter object in
  private Joystick m_joystickShooter; // Joystick variable
  private double m_lowShooterSpeed; // allows the low speed constant to be stored in a variable in this command

  public LowSpeedShooter(Joystick joystickShooter, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter; // stores shooter object from parameters
    m_joystickShooter = joystickShooter; // stores the joystickShooter object from parameters
  
    addRequirements(shooter); // If you get an nullPointerException, neutralize this
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lowShooterSpeed = Constants.lowShooterSpeed; // sets the low speed constant to the variable
    m_shooter.shooterSpeed(m_lowShooterSpeed); // calls the shooterSpeed method on the shooter object
  
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