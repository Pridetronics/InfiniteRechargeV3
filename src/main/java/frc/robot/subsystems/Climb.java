/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.print.event.PrintEvent;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DigitalInput;



public class Climb extends SubsystemBase {
  /**
   * Creates a new Climb.
   */

  private final DigitalInput limitSwitchDown = RobotContainer.limitSwitchDown;
  private final DigitalInput limitSwitchUp = RobotContainer.limitSwitchUp;


  public boolean limitSwitchUpOpen() {
    return limitSwitchUp.get();
    
  }

  public boolean limitSwitchDownOpen() {
    return limitSwitchDown.get();
  }


    public static CANSparkMax raiseClimbMotor;
    //public  static Talon telescopicClimbMotor;
    public  static CANSparkMax telescopicClimbMotor;

   
    public static  JoystickButton raiseTelescopic;
    public static JoystickButton descendTelescopic;
    public static JoystickButton sequenceClimbButton;

  public Climb() {
    //Lifts up, grabs bar, pulls self up

    raiseClimbMotor = RobotContainer.raiseClimbMotor;

    telescopicClimbMotor = RobotContainer.telescopicClimbMotor;

    

    //raiseTelescopic = new JoystickButton(shooterGamepad, 6);


    //descendTelescopic = new JoystickButton(shooterGamepad, 5);


    //sequenceClimbButton = new JoystickButton(shooterGamepad, 3);
  





  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public static void raiseClimbMotor() {
} //SequenceClimb uses this

}
