/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class GoDistance extends CommandBase {

  private Drive m_Robotdrive;
  private double setDistance;
  private CANEncoder leftDriveEncoder, rightDriveEncoder;
  private CANPIDController leftDrivePID, rightDrivePID;

  public GoDistance(double distance, Drive robotDrive) {
    // @param distance - Distance to travel in feet
    // @param robotDrive - Main drive system object
    m_Robotdrive = robotDrive;
    setDistance = distance;

    // Encoder Imports
    leftDriveEncoder = m_Robotdrive.leftDriveMotorEncoder;
    rightDriveEncoder = m_Robotdrive.rightDriveMotorEncoder;

    // PID Controller Imports
    leftDrivePID = m_Robotdrive.m_leftDrive_pid;
    rightDrivePID = m_Robotdrive.m_rightDrive_pid;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /* Enables the turning PID loop and sets the setpoint */
    /* The only reason we are using the turning PID loop here is to make sure the robot doesn't veer off to the side */
    m_Robotdrive.setSetpoint(m_Robotdrive.getMeasurement()); //Sets the setpoint to where the robot is currently looking
    m_Robotdrive.zeroRotationRate();
    m_Robotdrive.enable();

    /* Sets the encoder reference distance values to 0 */
    leftDriveEncoder.setPosition(0f);
    rightDriveEncoder.setPosition(0f);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftStickValue = Constants.AUTO_TRAVEL_SPEED + m_Robotdrive.getRotationRate(); // Adds the rotation rate to ensure the robot drives straight
    double rightStickValue = Constants.AUTO_TRAVEL_SPEED - m_Robotdrive.getRotationRate(); // Adds the rotation rate to ensure the robot drives straight
    m_Robotdrive.tankDrive(leftStickValue, rightStickValue, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /* Sets the encoder reference distance values to 0 because why not */
    leftDriveEncoder.setPosition(0f);
    rightDriveEncoder.setPosition(0f);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Don't need to convert setDistance from feet because of encoder position conversion factor set in RobotContainer
    double averageDistance = (leftDriveEncoder.getPosition() + rightDriveEncoder.getPosition()) / 2.0f;
    if (averageDistance >= setDistance) {
      return true;
    }
    else {
      return false;
    }
  }
}
