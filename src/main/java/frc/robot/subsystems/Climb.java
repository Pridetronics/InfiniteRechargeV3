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
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DigitalInput;



public class Climb extends SubsystemBase {
  /**
   * Creates a new Climb.
   */

    private DigitalInput limitLowerTelescopic;
    private DigitalInput limitUpperRobot;

    private static SpeedController robotClimbMotor;
    //public static Talon telescopicClimbMotor;
    private static SpeedController telescopicClimbMotor;

   
    private static JoystickButton raiseTelescopicButton;
    private static JoystickButton sequenceClimbButton;

  public Climb(SpeedController robotClimbMotor, SpeedController telescopicClimbMotor,
    DigitalInput limitLowerTelescopic, DigitalInput limitUpperRobot, JoystickButton raiseTelescopicButton, 
    JoystickButton sequenceClimbButton) {
    //CTOR
    //Lifts up, grabs bar, pulls self up

    this.robotClimbMotor = robotClimbMotor;

    this.telescopicClimbMotor = telescopicClimbMotor;

    this.limitLowerTelescopic = limitLowerTelescopic;

    this.limitUpperRobot = limitUpperRobot;
    
    this.raiseTelescopicButton = raiseTelescopicButton;

    this.sequenceClimbButton = sequenceClimbButton;


    //raiseTelescopic = new JoystickButton(shooterGamepad, 6);


    //descendTelescopic = new JoystickButton(shooterGamepad, 5);


    //sequenceClimbButton = new JoystickButton(shooterGamepad, 3);


  }

  public boolean getlimitLowerTelescopic() {
    return limitLowerTelescopic.get();
  }

  public boolean getlimitUpperRobot() {
    return limitUpperRobot.get();
  }

  //Function/Method

  public SpeedController getTelescopicClimbMotor() {
    return telescopicClimbMotor;
  }

  public SpeedController getRobotClimbMotor() {
    return robotClimbMotor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public static void robotClimbMotor() {
} //SequenceClimb uses this

}
