/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Joystick;

public class DriveJoystick extends CommandBase { //Creates a new DriveJoystick.
  
  private Joystick m_joystickDriver;
 
  public Drive m_robotDrive;

  private int driveMode;

  public DriveJoystick(Joystick joystickDriver, Drive robotDrive) {
    //driveMode = Robot.driveMode;
    // Use addRequirements() here to declare subsystem dependencies.
    m_joystickDriver = joystickDriver; // Grabs the joystick from RobotContainer
    m_robotDrive = robotDrive; //Grabs robotDrive from Drive
    
    addRequirements(robotDrive); 
    // Allows the default command (this file) to talk to the main subsystem
    // robotDrive is used as it's calls the differential drive in the Drive SUBSYSTEM
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driveMode == 0) 
    {
      double rightValue, leftValue; // Sets these up as doubles, allows to make it into axis
      if (m_robotDrive.arcadeModeOn()) {
        rightValue = m_joystickDriver.getRawAxis(4); // Right Joystick horizontal axis
        leftValue = m_joystickDriver.getRawAxis(1); // Left joystick verticle axis
        // If arcade mode is turned on in smart dashboard
        m_robotDrive.arcadeDrive(leftValue, rightValue, true, false);
      }
      else {
        rightValue = m_joystickDriver.getRawAxis(5); // Right Joystick verticle axis
        leftValue = m_joystickDriver.getRawAxis(1); // Left joystick verticle axis
        // If arcade mode is turned off in smart dashboard use tank drive
        m_robotDrive.tankDrive(leftValue, rightValue, true, false); 
      }
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
