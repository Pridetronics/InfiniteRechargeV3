/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class Climb extends SubsystemBase {
  /**
   * Creates a new Climb.
   */

    public CANSparkMax raiseClimbMotor;
    public Talon telescopicClimbMotor;

    public Joystick shooterGamepad;
    public JoystickButton raiseTelescopic;
    public JoystickButton descendTelescopic;
    public JoystickButton sequenceClimbButton;

  public Climb(Joystick joystickShooter) {
    //Lifts up, grabs bar, pulls self up

    raiseClimbMotor = new CANSparkMax(3, MotorType.kBrushed);
    raiseClimbMotor.setInverted(false);
    raiseClimbMotor.set(0);


    telescopicClimbMotor = new Talon(3);
    telescopicClimbMotor.setInverted(false);
    telescopicClimbMotor.set(0);

    shooterGamepad = new Joystick(1);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
