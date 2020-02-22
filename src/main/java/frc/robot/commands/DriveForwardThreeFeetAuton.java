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

  private CANEncoder leftDriveMotorEncoder;
  private CANEncoder rightDriveMotorEncoder;
 

  public DriveForwardThreeFeetAuton (int i, int j, Drive robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.

    rightDriveMotorEncoder = RobotContainer.rightDriveMotorLeadEncoder;
    leftDriveMotorEncoder = RobotContainer.leftDriveMotorLeadEncoder;
    m_robotDrive = robotDrive;
    addRequirements(robotDrive);

  }

  public DriveForwardThreeFeetAuton() {
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {

   rightDriveMotorEncoder.setPosition(0);
   leftDriveMotorEncoder.setPosition(0);

  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotDrive.tankDrive(-.95 , -.95);
    System.out.println(leftDriveMotorEncoder.getPosition());
    System.out.println(rightDriveMotorEncoder.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double averageDistance = leftDriveMotorEncoder.getPosition() + rightDriveMotorEncoder.getPosition()/2.0;
   if (averageDistance > 72){
    return false;
   }

    return true;
  }
}