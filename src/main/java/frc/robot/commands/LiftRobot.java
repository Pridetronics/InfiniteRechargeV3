/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climb;
import frc.robot.commands.ExtendTelescopicClimb;
import frc.robot.commands.DescendSequenceTelescopicClimb;
import frc.robot.commands.RaisesRobotClimb;


public class LiftRobot extends CommandBase {
  /**
   * Creates a new LiftRobot.
   * This is the second part of the Climb.
   */

   private static Climb climber;
   private static SequentialCommandGroup climbRobot;

  public LiftRobot(Climb aClimber) {
    //CTOR
    climber = aClimber;
    // Step 1: Motor lowers telescopic lift. 
    // Step 2: Then telescopic lift hits limit switch which switches over to step 3.
    // Step 3: A different motor then raises the robot.
    climbRobot = new SequentialCommandGroup(
      new DescendSequenceTelescopicClimb(climber), 
      new RaisesRobotClimb(raiseMotor)
      );
      
    // Use addRequirements() here to declare subsystem dependencies.
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
