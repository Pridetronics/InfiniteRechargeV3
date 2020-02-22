/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;

public class DriveTwoSeconds  extends CommandBase {
  private CANSparkMax leftDriveMotorLead;
  private CANSparkMax rightDriveMotorLead;

  private Drive m_robotDrive;

  public double startTime;
  /**
   * Creates a new Autonomous.
   */
  public DriveTwoSeconds (Drive robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    leftDriveMotorLead = RobotContainer.leftDriveMotorLead;
    rightDriveMotorLead = RobotContainer.rightDriveMotorLead;

    m_robotDrive = robotDrive;
    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = Timer.getFPGATimestamp();

    if (time - startTime < 1){
      leftDriveMotorLead.set(-0.25);
      rightDriveMotorLead.set(0.25);
    } else{
      leftDriveMotorLead.set(0);
      rightDriveMotorLead.set(0);
    }
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
