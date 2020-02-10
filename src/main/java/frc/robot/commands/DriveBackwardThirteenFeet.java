/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Encoder;


public class DriveBackwardThirteenFeet extends CommandBase {
  /**
   * Creates a new DriveBackwardThirteenFeet.
   */
  public Drive m_robotDrive;

  private double m_speedAuton;
  private double m_distanceAuton;
  double leftDriveMotorDistance;
  double rightDriveMotorDistance;

  private boolean stopRobot;

  private CANEncoder leftDriveMotorEncoder;
  private CANEncoder rightDriveMotorEncoder;

  public DriveBackwardThirteenFeet(double speedAuton, double distanceAuton, Drive robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_speedAuton = speedAuton;
    m_distanceAuton = distanceAuton;
    m_robotDrive = robotDrive;

    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    leftDriveMotorEncoder = Drive.getleftDriveMotorEncoder();
    rightDriveMotorEncoder = Drive.getrightDriveMotorEncoder();
    leftDriveMotorEncoder.setPosition(0);
    rightDriveMotorEncoder.setPosition(0);

    stopRobot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotDrive.tankDrive(m_speedAuton, m_distanceAuton);

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
