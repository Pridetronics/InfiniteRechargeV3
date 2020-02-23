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

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Drive extends PIDSubsystem { // Creates a new Drive.
  // NavX Import
  public AHRS navX;
  private double rotateToAngleRate = 0;

  public DifferentialDrive robotDrive;
  public MecanumDrive mecanumRobotDrive;
  public PIDController turnPID;
  public int driveMode;

  private CANSparkMax leftDriveMotor;
  private CANSparkMax rightDriveMotor;
  private CANEncoder leftDriveMotorEncoder;
  private CANEncoder rightDriveMotorEncoder;
 
  public Drive() {
    // Sets up the rotation PID controller
    super(new PIDController(Constants.TURN_kP, Constants.TURN_kI, Constants.TURN_kD));
    getController().setTolerance(Constants.TURN_TOLERANCE, Constants.TURN_PS_TOLERANCE); // Sets the tolerance to 5 degrees and the TPS tolerance to 10 degrees
    getController().enableContinuousInput(-180, 180); // Sets the controller to continuous because its an angle controller

    //NavX Setup
    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      navX = new AHRS(SPI.Port.kMXP); 
    } catch (RuntimeException ex) {
      // Failed to connect to the navX
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    // Sets up the drive motors
    leftDriveMotor = RobotContainer.leftDriveMotorLead; // references motors from RobotContainer
    rightDriveMotor =  RobotContainer.rightDriveMotorLead;
    
    // Encoder Setup
    leftDriveMotorEncoder = leftDriveMotor.getEncoder();
    rightDriveMotorEncoder = rightDriveMotor.getEncoder();
    /* Sets the gear ratio for the encoders */
    leftDriveMotorEncoder.setPositionConversionFactor(Constants.MAIN_MOTOR_RATIO);
    rightDriveMotorEncoder.setPositionConversionFactor(Constants.MAIN_MOTOR_RATIO);

    // DifferentialDrive Setup
    robotDrive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);
    robotDrive.setExpiration(0.1);    
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

  public void resetAngle() {
    /* Resets the yaw to 0 to wherever the robot is pointed */
    /* Only reccomended to press once at the start of the match to calibrate */
    navX.zeroYaw();
  }

  public double getRotationRate(){
    return rotateToAngleRate;
  }

  public void zeroRotationRate(){
    rotateToAngleRate = 0;
  }

    @Override
  public void useOutput(double output, double setpoint){
    rotateToAngleRate = output;
  }

    @Override
  public double getMeasurement() {
    return navX.getYaw();
  }
  
}
