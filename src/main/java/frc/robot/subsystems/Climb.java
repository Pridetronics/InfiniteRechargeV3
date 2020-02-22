/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase; // Imports needed for this subsystem
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;//Motor type (for competitions)

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;//Motor Type 
import com.revrobotics.CANSparkMaxLowLevel.MotorType; //Motor Type, specifically the certain CANSparkMaxes we will be using

import frc.robot.Constants; //Class -- These are currently unused
import frc.robot.Robot;// Class
import frc.robot.RobotContainer;//Class

public class Climb extends SubsystemBase 
{
  /**
   * Creates a new Climb.
   */
  private CANSparkMax m_spoolWinchMotor;
  private TalonSRX m_raiseRodMotor;
  private DigitalInput m_upperClimbLimitSwitch;
  private DigitalInput m_lowerClimbLimitSwitch;

  public Climb() 
  {
    m_raiseRodMotor = RobotContainer.raiseRodMotor;
    m_spoolWinchMotor = RobotContainer.spoolWinchMotor;
    m_upperClimbLimitSwitch = RobotContainer.upperClimbLimitSwitch;
    m_lowerClimbLimitSwitch = RobotContainer.lowerClimbLimitSwitch;
 
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  public void raiseTelescopicRod(double speed)
  {
    m_raiseRodMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void spoolWinch(double speed)
  {
    m_spoolWinchMotor.set(speed);
  }

  public boolean upperClimbLimitOpen()
  {
    return m_upperClimbLimitSwitch.get();
  }
  
  public boolean lowerClimbLimitOpen()
  {
    return m_lowerClimbLimitSwitch.get();
  }
}
