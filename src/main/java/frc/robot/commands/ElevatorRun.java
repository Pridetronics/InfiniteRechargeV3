/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;// Below adds the necessary imports for the command
import frc.robot.subsystems.Elevator;//Subsystem this command interfaces with
import edu.wpi.first.wpilibj.smartdashboard.*;//This is for the SmartDashboard to receive the values below
import com.revrobotics.CANSparkMax;//Motor Type
import edu.wpi.first.wpilibj.Talon;//Motor Type (for Competition)

public class ElevatorRun extends CommandBase {
  /**
   * Creates a new ElevatorRun.
   */
  //References the Elevator subsystem to grab the motors to be used for the commands
  private CANSparkMax elevatorMotorLead = Elevator.elevatorMotorLead;
  ///private Talon elevatorMotorLead = Elevator.elevatorMotorLead; //Competition
  private CANSparkMax elevatorMotorFollow = Elevator.elevatorMotorFollow; 
  ///private Talon elevatorMotorFollow = Elevator.elevatorMotorFollow; //Competition

  public ElevatorRun() {

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Elevator",  true);//Once this command is running, the dashboard will acknowledge it (Driver comfort?)
    elevatorMotorLead.set(55);//Once this command is executed the motor will run a little over half speed
  }
  public void ElevatorRun(){

    //SmartDashboard.putBoolean("Elevator",  true);
    //elevatorMotorLead.set(55);
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