/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pneumatics.ballReleasePiston;

public class CloseGate extends CommandBase {
  /**
   * Creates a new CloseGate.
   */
  private Joystick m_joystickShooter; // creates empty objects
  private Pneumatics m_pneumatics;
  //ballReleasePiston ballRelease;

  public CloseGate(Joystick joystickShooter, Pneumatics pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_joystickShooter = joystickShooter; // instantiates inside empty variables
    m_pneumatics = pneumatics;
    //ballRelease = ReleaseGate.ballRelease;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pneumatics.retractGate(); // retracts the gate
    ReleaseGate.ballRelease = ballReleasePiston.EXTENDED; // changes the state of the piston

  
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
