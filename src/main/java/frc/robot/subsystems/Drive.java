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

public class Drive extends SubsystemBase { // Creates a new Drive.
   
  public DifferentialDrive robotDrive;
  public int driveMode;

  private CANSparkMax leftDriveMotor;
  private CANSparkMax rightDriveMotor;
 
  public Drive(int leftDriveMotorAddress, int rightDriveMotorAddress) {
    // The ints inside the params of Drive () is called in RobotContainer
    driveMode = 0;
 
    leftDriveMotor = new CANSparkMax(leftDriveMotorAddress, MotorType.kBrushless);
    leftDriveMotor.setInverted(true); // Inverts Left Drive Motor
    leftDriveMotor.set(0); // Sets speed to 0 (anywhere between -1 and 1)

    rightDriveMotor = new CANSparkMax(rightDriveMotorAddress, MotorType.kBrushless); // Assigns Leading Right Drive Motor to Talon #2
    rightDriveMotor.setInverted(true); // Inverts Right Drive Motor
    rightDriveMotor.set(0); // Sets speed to 0 (anywhere between -1 and 1)
    
    robotDrive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);
    // Constructs the differential drive with the motors

    SmartDashboard.putString("Drive Mode:", "Tank");                                                                                                              
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


  public void tankDrive(double leftValue, double rightValue) {
    // This method was not here, it was created to run the axis values in DriveJoystick
    // Defines the variables so that way tank can work
    // Located in Drive because the drivetrain is grabbed from this subsystem
    
    robotDrive.tankDrive(leftValue, rightValue); // Grabs the raw axis from DriveJoystick    
  }
  
}
