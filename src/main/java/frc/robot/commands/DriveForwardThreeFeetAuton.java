/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.revrobotics.CANEncoder;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveForwardThreeFeetAuton extends CommandBase {
  /**
   * Creates a new DriveForwardTwoFeetAuton.
   */
  public Drive m_robotDrive;

  private double m_speedAuton;
  private double m_distanceAuton;
  double leftDriveMotorDistance;
  double rightDriveMotorDistance;
  private CANEncoder leftDriveMotorEncoder;
  private CANEncoder rightDriveMotorEncoder;
  public int distanceStopAuton;
  private boolean stopRobot;

  public DriveForwardThreeFeetAuton(double speedAuton, double distanceAuton, Drive robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_speedAuton = speedAuton;
    m_distanceAuton = distanceAuton;
    distanceAuton = 3;

  
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
    m_robotDrive.tankDrive(1, 2);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.tankDrive(0,0);
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double leftDriveMotorDistance = m_robotDrive.getleftDriveMotorEncoder().getPosition();
    double rightDriveMotorDistance = m_robotDrive.getrightDriveMotorEncoder().getPosition();
   
    
    double averageDistance = (Math.abs(leftDriveMotorDistance) + Math.abs(rightDriveMotorDistance)) / 2;

    SmartDashboard.putNumber("Distance", averageDistance);

    if (averageDistance >= m_distanceAuton || stopRobot){
      return true;
    }

    return false;
  }
}
