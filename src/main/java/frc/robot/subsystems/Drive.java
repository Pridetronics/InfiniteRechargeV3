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
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.PIDController;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SpeedController;

public class Drive extends SubsystemBase { // Creates a new Drive.
  // NavX Import
  public AHRS navX;

  public DifferentialDrive robotDrive;
  public MecanumDrive mecanumRobotDrive;
  public PIDController turnPID;
  public int driveMode;

  private CANSparkMax leftDriveMotor;
  private CANSparkMax rightDriveMotor;
 
  public Drive() {
    // The ints inside the params of Drive () is called in RobotContainer
    driveMode = 0;
    //NavX Setup
    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      navX = new AHRS(SPI.Port.kMXP); 
    } catch (RuntimeException ex) {
      // Failed to connect to the navX
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    leftDriveMotor = RobotContainer.leftDriveMotorLead; // references motors from RobotContainer
    rightDriveMotor =  RobotContainer.rightDriveMotorLead;
    
    // Drive Setup
    robotDrive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);
    robotDrive.setExpiration(0.1);

    // Auto Drive Setup
    turnPID = new PIDController(Constants.TURN_kP, Constants.TURN_kI, Constants.TURN_kD);


    // Puts the drive mode on the smart dashboard
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

  public void autoDrive() {
    

  }

  
}
