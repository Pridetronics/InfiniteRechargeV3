/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase; // Imports needed for this subsystem
import edu.wpi.first.wpilibj.Talon;//Motor type (for competitions)
import com.revrobotics.CANSparkMax;//Motor Type 
import com.revrobotics.CANSparkMaxLowLevel.MotorType; //Motor Type, specifically the certain CANSparkMaxes we will be using

import frc.robot.Constants; //Class -- These are currently unused
import frc.robot.Robot;// Class
import frc.robot.RobotContainer;//Class

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  public static CANSparkMax intakeMotor = RobotContainer.intakeMotor; // Testing Motor: Creates an intake motor 
  
   ///private Talon intakeMotor = RobotContainer.intakeMotor; //Competition Motor--> Intake motor

   public Intake() {
    //Collect power cell balls
    
   }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
