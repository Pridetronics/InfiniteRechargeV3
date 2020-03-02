/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.PIDController;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Drive extends PIDSubsystem { // Creates a new Drive.
  // NavX Declaration
  public AHRS navX;

  // Rotation PID Setup
  private double rotateToAngleRate = 0;

  // Differential drive setup, currently not in use
  public DifferentialDrive robotDrive;

  // NEO motors declaration
  private CANSparkMax leftDriveMotor;
  private CANSparkMax rightDriveMotor;
  
  // Encoder Declaration
  private CANEncoder leftDriveEncoder;
  private CANEncoder rightDriveEncoder;
  
  // Velocity PID loops for the motors
  public CANPIDController m_leftDrive_pid;
  public CANPIDController m_rightDrive_pid;

  // PID Constants for the velocity PID loop, not for direction
  private double kP, kI, kD;

  // Odometry Setup for Pathing
  private DifferentialDriveOdometry m_odometry;
 
  public Drive() {
    // Sets up the rotation PID controller
    super(new PIDController(Constants.TURN_kP, Constants.TURN_kI, Constants.TURN_kD));
    getController().setTolerance(Constants.TURN_TOLERANCE, Constants.TURN_PS_TOLERANCE); // Sets the tolerance to 5 degrees and the TPS tolerance to 10 degrees
    getController().enableContinuousInput(-180, 180); // Sets the controller to continuous because its an angle controller
    disable(); // Disables the PID loop

    // Connect the NAVX to the port on the RoboRIO
    try {
      /* Communicate with the navX-MXP via the MXP SPI Bus */
      navX = new AHRS(SPI.Port.kMXP); 
    } catch (RuntimeException ex) {
      // Failed to connect to the navX
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    // Sets up the drive motors
    leftDriveMotor = RobotContainer.leftDriveMotorLead; // references motors from RobotContainer
    rightDriveMotor =  RobotContainer.rightDriveMotorLead;

    // Sets up the motor encoders
    leftDriveEncoder = RobotContainer.leftDriveEncoder;
    rightDriveEncoder = RobotContainer.rightDriveEncoder;
    resetEncoders();

    // Sets up the odometry
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(navX.getYaw()));

    // PID Setup
    kP = Constants.DRIVE_kP;
    kI = Constants.DRIVE_kI;
    kD = Constants.DRIVE_kD;
    m_leftDrive_pid = RobotContainer.leftDrive_pid;
    m_rightDrive_pid = RobotContainer.rightDrive_pid;

    // PID coefficients display on SmartDashboard
    SmartDashboard.putNumber("Drive P Gain", kP);
    SmartDashboard.putNumber("Drive I Gain", kI);
    SmartDashboard.putNumber("Drive D Gain", kD);

    // Sets up the robot DifferentialDrive object, currently not in use    
    robotDrive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);
    robotDrive.setExpiration(0.1);
  }

  public void initDefaultCommand() {
    // setDefaultCommand(new DriveTeleop());
  }


  @Override
  public void periodic() {
    // Updates the odometry object with new position info
    /* Need to update Encoders to output distance in meters instead of feet to work */
    m_odometry.update(Rotation2d.fromDegrees(getMeasurement()), leftDriveEncoder.getPosition(),rightDriveEncoder.getPosition());

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Drive P Gain", 0);
    double i = SmartDashboard.getNumber("Drive I Gain", 0);
    double d = SmartDashboard.getNumber("Drive D Gain", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_leftDrive_pid.setP(p); m_rightDrive_pid.setP(p); kP = p; }
    if((i != kI)) { m_leftDrive_pid.setI(i); m_rightDrive_pid.setI(i); kI = i; }
    if((d != kD)) { m_leftDrive_pid.setD(d); m_rightDrive_pid.setD(d); kD = d; }
  }

  protected double applyDeadband(double value, double deadband)
  {
    // Deadzone function for the tankDrive
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
    // Replaces the regular tankdrive with a PID loop that controls the RPM
    // This compensates for the amperage drop in the battery and makes it much smoother
    
    /* Left this here for testing, it's the original tankdrive function */
    // robotDrive.tankDrive(leftValue, rightValue, squareInputs);

    // Checks that the value is between -1 and 1
    leftValue = MathUtil.clamp(leftValue, -1.0, 1.0);
    rightValue = MathUtil.clamp(rightValue, -1.0, 1.0);
    
    // Creates a deadzone on the controller to reduce drive jitter
    leftValue = applyDeadband(leftValue, Constants.DEADBAND);
    rightValue = applyDeadband(rightValue, Constants.DEADBAND);

    // Squares the input to make it a exponential response curve instead of linear
    // to increase fine control while permitting full power
    if (squareInputs) 
    {
      // Squares the values and copies the sign from the initial value
      // This makes sure that if the values were negative that they stay negative after the square
      // Take a look at the intensity of the squaring of the inputs
      leftValue = Math.copySign(leftValue * leftValue, leftValue);
      rightValue = Math.copySign(rightValue * rightValue, rightValue);
    }
    
    // Converts the percentage value to RPM for the PID Loop
    leftValue *= Constants.MAX_NEO_RPM;
    rightValue *= Constants.MAX_NEO_RPM;

    // Sets the reference point on the PID loop to the specified RPM
    m_leftDrive_pid.setReference(leftValue, ControlType.kVelocity);
    m_rightDrive_pid.setReference(rightValue, ControlType.kVelocity);

    // Feeds the safety object with the new robot data
    robotDrive.feed();
  }

  /* Trajectory Methods */

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    // tankDrive method that is used for trajectory generation
    // Used since the RamsetteController gives output in volts
    leftDriveMotor.setVoltage(leftVolts);
    rightDriveMotor.setVoltage(-rightVolts);
    robotDrive.feed();
  }

  public Pose2d getPose() {
    // Returns a Pose2D object from the odometry
    return m_odometry.getPoseMeters();
  }

  /* Utility Methods */

  public void resetAngle() {
    /* Resets the yaw to 0 to wherever the robot is pointed */
    navX.zeroYaw();
  }

  public double getRotationRate(){
    // Gets the rotation rate from the rotation PID loop
    return rotateToAngleRate;
  }

  public void zeroRotationRate(){
    // Zeroes the rotation rate for the PID loop
    rotateToAngleRate = 0;
  }

  public void resetEncoders() {
    // Resets the encoder distance to 0
    leftDriveEncoder.setPosition(0.0);
    rightDriveEncoder.setPosition(0.0);
  }

  public boolean atSetPoint() {
    // Returns true if the PID loop is at its setpoint within the set tolerances
    return getController().atSetpoint();
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
