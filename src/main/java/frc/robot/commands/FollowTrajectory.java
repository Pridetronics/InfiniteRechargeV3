/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class FollowTrajectory extends RamseteCommand {
  // @param robotDrive - Drive subsystem
  // @param trajectory - Actual trajectory to follow
  DifferentialDriveKinematics driveKinematics;
  TrajectoryConfig trajectoryConfig;
  Drive m_Robotdrive;
  Trajectory driveTrajectory;

  public FollowTrajectory(Drive robotDrive, Trajectory trajectory) {
    // Sets up the Ramsette Command
    super(
    trajectory,
    robotDrive::getPose,
    new RamseteController(Constants.RAMSETTE_B, Constants.RAMSETTE_Z),
    robotDrive.driveFeedforward,
    robotDrive.driveKinematics,
    robotDrive::getWheelSpeeds,
    new PIDController(Constants.SPEC_DRIVE_VELOCITY, 0, 0),
    new PIDController(Constants.SPEC_DRIVE_VELOCITY, 0, 0),
    robotDrive::tankDriveVolts,
    robotDrive
    );
    
    // Parameter Imports
    m_Robotdrive = robotDrive;
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Robotdrive.tankDriveVolts(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isFinished()) {
      return true;
    }
    else {
      return false;
    }
  }
}
