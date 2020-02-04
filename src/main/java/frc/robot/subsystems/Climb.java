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

    public static CANSparkMax raiseClimbMotor;
    //public  static Talon telescopicClimbMotor;
    public  static CANSparkMax telescopicClimbMotor;

   
    public static  JoystickButton raiseTelescopic;
    public static JoystickButton descendTelescopic;
    public static JoystickButton sequenceClimbButton;

  public Climb() {
    //Lifts up, grabs bar, pulls self up

    raiseClimbMotor = new CANSparkMax(1, MotorType.kBrushed);
    raiseClimbMotor.setInverted(false);
    raiseClimbMotor.set(0);

    //telescopicClimbMotor = new Talon(1);
    telescopicClimbMotor = new CANSparkMax(2, MotorType.kBrushed);
    telescopicClimbMotor.setInverted(false);
    telescopicClimbMotor.set(0);


    //raiseTelescopic = new JoystickButton(shooterGamepad, 6);


    //descendTelescopic = new JoystickButton(shooterGamepad, 5);


    //sequenceClimbButton = new JoystickButton(shooterGamepad, 3);
  





  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
