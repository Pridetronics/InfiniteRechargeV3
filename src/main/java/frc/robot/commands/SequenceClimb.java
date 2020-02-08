/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.revrobotics.CANSparkMax;

import frc.robot.RobotContainer;

import frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

public class SequenceClimb extends CommandBase {
  /**
   * Creates a new SequenceClimb.
   */

  public final DigitalInput limitSwitchDown = RobotContainer.limitSwitchDown;
  public final DigitalInput limitSwitchUp = RobotContainer.limitSwitchUp;

  public static CANSparkMax raiseClimbMotor = Climb.raiseClimbMotor;



  public SequenceClimb(CANSparkMax raiseMotor) {
    // Use addRequirements() here to declare subsystem dependencies.
 
    raiseClimbMotor = raiseMotor;
    
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Climb.raiseClimbMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
/*  int output = JoystickButton.getY(); //Moves the joystick based on Y value
    if (limitSwitchDown closed) 
    limitSwitchUp.get();
     //When pressed, value will be between -1 and 0
     output = Math.min(output, 0);
  }
  else {
   limitSwitchDown.get(); //When pressed value with be between 0 and 1
    output = Math.max(output, 0);
    raiseClimbMotor.set(output);
*/
  raiseClimbMotor.set(.55);
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



  //reeeeeeeeeeeeeeeee
}
