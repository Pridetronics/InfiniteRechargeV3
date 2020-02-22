/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Commands.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SpeedController;

import com.revrobotics.CANEncoder;

public class Drive extends SubsystemBase { // Creates a new Drive.
   
  public DifferentialDrive robotDrive;
  public int driveMode;

  private CANSparkMax leftDriveMotor;
  private CANSparkMax rightDriveMotor;

  public static CANEncoder leftDriveMotorEncoder = RobotContainer.leftDriveMotorLeadEncoder;
  public static CANEncoder rightDriveMotorEncoder = RobotContainer.rightDriveMotorLeadEncoder;
  

 
  public Drive() {
    // The ints inside the params of Drive () is called in RobotContainer
    driveMode = 0;
 
    leftDriveMotor = RobotContainer.leftDriveMotorLead; // references motors from RobotContainer
    rightDriveMotor =  RobotContainer.rightDriveMotorLead;

    robotDrive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);
    // Constructs the differential drive with the motors
    rightDriveMotorEncoder.setPositionConversionFactor(2.579);
    leftDriveMotorEncoder.setPositionConversionFactor(2.579);
    
    //SmartDashboard.putString("Drive Mode:", "Tank");                                                                                                              
  }


public void setDrive() {
    /*
    if (driveMode == 0) {
      driveMode = 1;
      SmartDashboard.putString("Drive Mode", "Arcade");
    } 

    
    else {
      driveMode = 0;
      SmartDashboard.putString("Drive Mode", "Tank");
    }
    */

    
  }


  public void initDefaultCommand() {
    // setDefaultCommand(new DriveTeleop());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void doTeleop() {    
    //test 1
    
  }

  public static CANEncoder getleftDriveMotorEncoder(){
    return leftDriveMotorEncoder;
  }

  public static CANEncoder getrightDriveMotorEncoder(){
    return rightDriveMotorEncoder;
  }

  public void tankDrive(double leftValue, double rightValue) {
    // This method was not here, it was created to run the axis values in DriveJoystick
    // Defines the variables so that way tank can work
    // Located in Drive because the drivetrain is grabbed from this subsystem
    rightValue = driveSquare(rightValue, .5) * .75;
    leftValue = driveSquare(leftValue, .5) * .75;
    robotDrive.tankDrive(leftValue, rightValue); // Grabs the raw axis from DriveJoystick   
    // System.out.println(leftDriveMotorLeadEncoder.getCountsPerRevolution());
    // System.out.println(rightDriveMotorLeadEncoder.getCountsPerRevolution());

  }
  
  public double driveSquare(double input, double degree) {
    double a = .2;
    return (Math.pow(input, 3) + a * (degree * input)) / (a * degree + 1); //Document Later
  }
  
}
