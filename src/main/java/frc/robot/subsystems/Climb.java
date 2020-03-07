/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase; // Imports needed for this subsystem
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;//Motor Type 
import frc.robot.RobotContainer;//Class

public class Climb extends SubsystemBase 
{
  /**
   * Creates a new Climb.
   */
  //motor objects and limit switches
  private CANSparkMax m_spoolWinchMotor;
  private TalonSRX m_raiseRodMotor;
  private DigitalInput m_upperClimbLimitSwitch;
  private DigitalInput m_lowerClimbLimitSwitch;

  public Climb() 
  {
    m_raiseRodMotor = RobotContainer.raiseRodMotor; // motor to raise telescopic rod
    m_spoolWinchMotor = RobotContainer.spoolWinchMotor; // motor to raise the robot
    m_upperClimbLimitSwitch = RobotContainer.upperClimbLimitSwitch; // limit switch to test if telescopic rod is extended
    m_lowerClimbLimitSwitch = RobotContainer.lowerClimbLimitSwitch; // limit switch to test if telescopic rod has descended
 
  }
// 0 is closed, 1 is open
  @Override
  public void periodic() 
  {
    // PID coefficients display on SmartDashboard
    SmartDashboard.putBoolean("Upper Climb Limit Switch", m_upperClimbLimitSwitch.get());
    SmartDashboard.putBoolean("Lower Climb Limit Switch", m_lowerClimbLimitSwitch.get());
  }

  public void raiseTelescopicRod(double speed)
  {
    // @param speed - speed the telescopic rod moves at
    m_raiseRodMotor.set(TalonSRXControlMode.PercentOutput, speed); // sets the motor at 20% speed
  }

  public void spoolWinch(double speed)
  {
    // @param speed - speed the winch motor moves at
    m_spoolWinchMotor.set(speed); // sets the motor at 20% speed
  }

  public boolean isClimbAtTop()
  {
    //Returns the state of the upper limit switch
    boolean isClimbAtTop;
    if(m_upperClimbLimitSwitch.get())
    {
      isClimbAtTop = false;
    }
    else
    {
      isClimbAtTop = true;
    }
    return isClimbAtTop;
  }
  
  public boolean isClimbAtBottom()
  {
    //Returns the state of the lower limit switch
    return (!m_lowerClimbLimitSwitch.get());
  }
}
