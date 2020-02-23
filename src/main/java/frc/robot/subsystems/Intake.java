/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase; // Imports needed for this subsystem
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.CANSparkMax;//Motor Type 
import frc.robot.RobotContainer;//Class

/*
  0,1 for intake solenoid. 2,3 for shooter solenoid
*/
public class Intake extends SubsystemBase 
{
  /**
   * Creates a new Intake.
   */
  private CANSparkMax m_intakeMotor;
  private DoubleSolenoid m_intakeExtendRetract;
  
   ///private Talon intakeMotor = RobotContainer.intakeMotor; //Competition Motor--> Intake motor

   public Intake() 
   {
    //Collect power cell balls
    m_intakeMotor = RobotContainer.intakeMotor; // Testing Motor: Creates an intake motor
    m_intakeExtendRetract = RobotContainer.intakeExtendRetract;

   }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  public void runIntakeMotors(double speed)
  {
    m_intakeMotor.set(speed);
  }

  public void extendIntake()
  {
    m_intakeExtendRetract.set(DoubleSolenoid.Value.kForward);
  }

  public void retractIntake()
  {
    m_intakeExtendRetract.set(DoubleSolenoid.Value.kReverse);
  }
}
