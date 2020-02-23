/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

public class Drive extends SubsystemBase { // Creates a new Drive.
   
  public DifferentialDrive robotDrive;
  public int driveMode;

  private CANSparkMax leftDriveMotor;
  private CANSparkMax rightDriveMotor;

  private CANPIDController m_leftDrive_pid;
  private CANPIDController m_rightDrive_pid;

  private double leftDriveMotorRPM;
  private double rightDriveMotorRPM;
 
  public Drive() {
    // The ints inside the params of Drive () is called in RobotContainer
    driveMode = 0;
 
    leftDriveMotor = RobotContainer.leftDriveMotorLead; // references motors from RobotContainer
    rightDriveMotor =  RobotContainer.rightDriveMotorLead;

    m_leftDrive_pid = RobotContainer.leftDrive_pid;
    m_rightDrive_pid = RobotContainer.rightDrive_pid;
    
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

  protected double applyDeadband(double value, double deadband)
  {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public void tankDrive(double leftValue, double rightValue, boolean squareInputs) {
    // This method was not here, it was created to run the axis values in DriveJoystick
    // Defines the variables so that way tank can work
    // Located in Drive because the drivetrain is grabbed from this subsystem
    
    //robotDrive.tankDrive(leftValue, rightValue); // Grabs the raw axis from DriveJoystick
    // Checks that the value is between -1 and 1
    leftValue = MathUtil.clamp(leftValue, -1.0, 1.0);
    rightValue = MathUtil.clamp(rightValue, -1.0, 1.0);
    
    // Creates a deadzone on the controller to reduce drive jitter
    leftValue = applyDeadband(leftValue, Constants.DEADBAND);
    rightValue = applyDeadband(rightValue, Constants.DEADBAND);

    // Squares the input to make it a exponential response curve instead of linear
    // to increase fine control while permitting full power.
    if (squareInputs) 
    {
      leftValue = Math.copySign(leftValue * leftValue, leftValue);
      rightValue = Math.copySign(rightValue * rightValue, rightValue);
    }
    
    // Converts the percentage value to RPM for the PID Loop
    leftDriveMotorRPM = leftValue * Constants.MAX_SHOOTER_RPM;
    rightDriveMotorRPM = rightValue * Constants.MAX_SHOOTER_RPM;

    // Sets the reference point on the PID loop to the specified RPM
    m_leftDrive_pid.setReference(leftDriveMotorRPM, ControlType.kVelocity);
    m_rightDrive_pid.setReference(rightDriveMotorRPM, ControlType.kVelocity);
  }
  
}
