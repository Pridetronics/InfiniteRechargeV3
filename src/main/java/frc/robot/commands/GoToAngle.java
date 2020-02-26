/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drive;

public class GoToAngle extends CommandBase {
  
  private Drive m_Robotdrive;
  private double setAngle;

  public GoToAngle(double angle, Drive robotDrive) {
    // @param angle - Angle to turn to in degrees
    // @param robotDrive - Drive system subclass

    // Imports the parameters
    setAngle = angle;
    m_Robotdrive = robotDrive;

    addRequirements(m_Robotdrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /* Enables the turning PID loop and sets the setpoint */
    m_Robotdrive.setSetpoint(setAngle);
    m_Robotdrive.zeroRotationRate();
    m_Robotdrive.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Puts the stick values to the rotation rate and drives the robot */
    /* Will only rotate the robot to the specified value */
    double leftStickValue = m_Robotdrive.getRotationRate();
    double rightStickValue = -m_Robotdrive.getRotationRate();
    m_Robotdrive.tankDrive(leftStickValue, rightStickValue, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Disables the PID loop
    m_Robotdrive.disable();
    m_Robotdrive.resetAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Only terminate the command once the angle is set
    return m_Robotdrive.atSetPoint();
  }
}
