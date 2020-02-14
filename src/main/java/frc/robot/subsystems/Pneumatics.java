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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class Pneumatics extends SubsystemBase {
  /**
   * Creates a new Pneumatics.
   */
  
  // empty double solenoids to bring in from RobotContainer
  private final DoubleSolenoid shooterBallRelease; 
  private final DoubleSolenoid intakeDeploy;
  private final DoubleSolenoid controlPanelSpinnerDeploy;

  /*
  enum(enumerated type) allows you to create your own data type and create a fixed set of constants
  for that data type. To reference enums in another class, you have to import it. Using enums to represent 
  the state of a piston is not helpful now because we only have pistons that can only extend and retract, but 
  using them will be helpful in the future when we have pistons that could have more states than just 
  extended or retracted, such as half-extended.
  */
  public enum ballReleasePiston 
  {
    EXTENDED, RETRACTED;
  }
  
   public Pneumatics() {
    // sets the double solenoids equal to the double solenoids form RobotContainer
    shooterBallRelease = RobotContainer.shooterBallRelease; 
    intakeDeploy = RobotContainer.intakeDeploy;
    controlPanelSpinnerDeploy = RobotContainer.controlPanelSpinnerDeploy;
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void releaseGate() // This method will release the gate
  {
    // This reverses the air flow, which should release the gate
    System.out.println(Value.kReverse);
    shooterBallRelease.set(DoubleSolenoid.Value.kReverse);
  }

  public void retractGate() // This method will bring the gate back up again
  {
    //This lets the air go through, which should close the gate
    shooterBallRelease.set(Value.kForward);
  }
}
