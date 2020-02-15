/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class RaisesRobotClimb extends CommandBase {
  /**
   * Creates a new RisesRobotClimb.
   */
  private static Climb robotClimbMotor;

  public RaisesRobotClimb(Climb raiseMotor) {
    // Use addRequirements() here to declare subsystem dependencies.
    robotClimbMotor = raiseMotor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotClimbMotor.getRobotClimbMotor().set(.55);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotClimbMotor.getRobotClimbMotor().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(robotClimbMotor.getlimitUpperRobot()){
      return true;
    }
    else{
    return false;
    }
  }
}
