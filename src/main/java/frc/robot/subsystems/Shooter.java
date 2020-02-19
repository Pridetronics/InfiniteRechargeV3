/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import frc.robot.subsystems.RobotContainer;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private CANSparkMax shooterMotor; // creates a new motor variable
  private final DoubleSolenoid m_shooterBallRelease; 
  
  public Shooter() {
    //Launches power cells (balls) into the goals (levels 1, 2, and 3).
    shooterMotor = RobotContainer.shooterMotor; // references shooter motor from RobotContainer
    m_shooterBallRelease = RobotContainer.shooterBallRelease;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /*allows the speed of the motor to be set and then run the motor at that speed
  */
  public void shooterSpeed(double speed)
  {
      double shooterSpeed = speed; // declares a variable that is set to the speed parameter
      shooterMotor.set(shooterSpeed); // runs the motor at the speed of the parameter

  }

  public void releaseGate() // This method will release the gate
  {
    // This reverses the air flow, which should release the gate
    m_shooterBallRelease.set(DoubleSolenoid.Value.kForward);
  }

  public void retractGate() // This method will bring the gate back up again
  {
    //This lets the air go through, which should close the gate
    m_shooterBallRelease.set(DoubleSolenoid.Value.kReverse);
  }
}

