/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

import frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj.Talon;

import com.revrobotics.CANSparkMax;

public class DescendSequenceTelescopicClimb extends CommandBase {
  /**
   * Creates a new DescendSequenceTelescopicClimb.
   */
  public static CANSparkMax telescopicClimbMotor = Climb.telescopicClimbMotor;

  public DescendSequenceTelescopicClimb(CANSparkMax DescendSequenceMotor) {
    // Use addRequirements() here to declare subsystem dependencies.
    telescopicClimbMotor = DescendSequenceMotor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    telescopicClimbMotor.set(-.55);
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
