/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase; // Imports needed for this subsystem
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
  private TalonSRX m_intakeMotor;
  private DoubleSolenoid m_intakeExtendRetract;

   public Intake() 
   {
    //Brings in the intake motor and the intake double solenoid
    m_intakeMotor = RobotContainer.intakeMotor;
    m_intakeExtendRetract = RobotContainer.intakeExtendRetract;
   }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  //Runs the intake motors
  public void runIntakeMotors(double speed)
  {
    //@param - Speed in Percentages
    m_intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  //Extends the intake
  public void extendIntake()
  {
    m_intakeExtendRetract.set(DoubleSolenoid.Value.kReverse);
  }

  //Retracts the intake
  public void retractIntake()
  {
    m_intakeExtendRetract.set(DoubleSolenoid.Value.kForward);
  }
}
