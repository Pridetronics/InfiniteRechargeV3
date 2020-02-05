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
  public static CANSparkMax intakeMotorExternal; // Testing Motor: Creates an intake motor for the external intake
  public static CANSparkMax intakeMotorVerticalLead; //Testing Motor: Creates an intake motor for the vertical intake (leading motor)
  public static CANSparkMax intakeMotorVerticalFollow; //Testing Motor: Creates an intake motor to follow the vertical lead motor
  ///private Talon intakeMotorExternal; //Competition Motor--> Intake motor for external intake created
  ///private Talon intakeMotorVerticalLead; //Competition Motor-->Intake motor for vertical intake created(LEAD)
  ///private Talon intakeMotorVerticalFollow;//Competition Motor--> Intake motor for vertical intake created (FOLLOW)

   public Intake() {
    //Collect power cell balls
    intakeMotorExternal = new CANSparkMax(5, MotorType.kBrushless); //The motor (CANSparkMax) is defined with a type and port (port 5, and motor type = brushless)
    ///intakeMotorExternal =  new Talon(5); //Motor is defined as a specified motor under port five (Talon)
    intakeMotorExternal.set(0); //Initially sets motor value to 0, will not run without further command

    intakeMotorVerticalLead = new CANSparkMax(6, MotorType.kBrushless); //Motor is defined under 6th port (CANSparkMax)
    ///intakeMotorVerticalLead = new Talon(6); //Motor is defined (Talon) under port six
    intakeMotorVerticalLead.set(0); //Sets motor value (speed) to 0

    intakeMotorVerticalFollow = new CANSparkMax(7, MotorType.kBrushless); //Motor is deinfed under 7th port (CANSparkMax)
    ///intakeMotorVerticalFollow = new Talon(7); // Motor is defined under port seven (Talon)
    intakeMotorVerticalFollow.set(0); //Sets motor speed to 0
    intakeMotorVerticalFollow.follow(intakeMotorVerticalLead); // Vertical follow motor will do everthing the vertical lead motor does
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
