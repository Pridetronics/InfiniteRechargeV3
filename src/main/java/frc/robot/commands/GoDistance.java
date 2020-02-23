/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drive;

public class GoDistance extends CommandBase {

  private Drive m_Robotdrive;
  private double setDistance;

  public GoDistance(double distance, Drive robotDrive) {
    // @param distance - Distance to travel in feet
    // @param robotDrive - Main drive system object
    m_Robotdrive = robotDrive;
    setDistance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /* Converts the Distance in Feet -> Revolutions */
    


    /* Enables the turning PID loop and sets the setpoint */
    /* The only reason we are using the turning PID loop here is to make sure the robot doesn't veer off to the side */
    m_Robotdrive.setSetpoint(m_Robotdrive.getMeasurement()); //Sets the setpoint to where the robot is currently looking
    m_Robotdrive.zeroRotationRate();
    m_Robotdrive.enable();

    /* Imports the Encoders for distance checking */
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
