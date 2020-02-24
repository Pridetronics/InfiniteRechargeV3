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
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

public class Shooter extends SubsystemBase 
{
  /**
   * Creates a new Shooter.
   */
  // private CANSparkMax shooterMotor; // creates a new motor variable
  private final DoubleSolenoid m_shooterBallRelease; //double solenoid for shooter gate
  private CANPIDController shooter_pid; // pid controller for our shooter pid control loop
  
  public Shooter() 
  {
    //Launches power cells (balls) into the goals (levels 1, 2, and 3).
    // shooterMotor = RobotContainer.shooterMotor; // references shooter motor from RobotContainer
    //brings in pid controller and double solenoids for robot container
    shooter_pid = RobotContainer.shooterMotor_pid;
    m_shooterBallRelease = RobotContainer.shooterBallRelease;
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
  /*allows the speed of the motor to be set and then run the motor at that speed
  */
  public void shooterSpeed(double speed)
  {
      // @param speed - Speed in RPM's
      double shooterSpeed = speed; // declares a variable that is set to the speed parameter
      shooter_pid.setReference(shooterSpeed, ControlType.kVelocity); // sets the PID loop to the speed under the Velocity type
      // shooterMotor.set(shooterSpeed); // runs the motor at the speed of the parameter, old

  }

  public void releaseGate() // This method will release the gate
  {
    // This reverses the air flow, which should release the gate
    m_shooterBallRelease.set(DoubleSolenoid.Value.kForward);
  }

  public void retractGate() // This method will bring the gate back up again
  {
    //This lets the air release, which should close the gate
    m_shooterBallRelease.set(DoubleSolenoid.Value.kReverse);
  }
}

