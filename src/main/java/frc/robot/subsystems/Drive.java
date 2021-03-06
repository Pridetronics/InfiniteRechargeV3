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
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

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

  // Last Input Storage for Input Ramping
  private double leftLastInput;
  private double rightLastInput;

  // Stores input of stick values
  private double leftStickValue;
  private double rightStickValue;

  // Drive Modes from Tank to Arcade to Curvature
  private boolean arcadeMode;
  private boolean curvatureMode;

  // Odometry Setup for Pathing
  public DifferentialDriveOdometry driveOdometry;

  // DifferentialDriveKinematics Object for Trajectory Calculations
  public DifferentialDriveKinematics driveKinematics;

  // Simple Feed Forward Declaration
  public SimpleMotorFeedforward driveFeedforward;
 
  // Differential Drive Voltage Constraint Instantiation
  public DifferentialDriveVoltageConstraint driveVoltageConstraint;

  // Trajectory Config Instantiation
  public TrajectoryConfig trajectoryConfig;

  public Drive() {
    /* Subsystem Methods */
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
    driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(navX.getYaw()));

    // Sets up the DifferentialDriveKinematics object
    driveKinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);

    // Sets up the SimpleMotorFeedforward for use in the Ramsete Controller and the Voltage Constraint
    driveFeedforward = new SimpleMotorFeedforward(Constants.SPEC_VOLTS, Constants.SPEC_VOLT_SECONDS_PER_METER, Constants.SPEC_VOLT_SECONDS_SQUARE_PER_METER);

    // Sets up the voltage constraint
    driveVoltageConstraint = new DifferentialDriveVoltageConstraint(driveFeedforward, driveKinematics, 10);

    // Sets up the trajectory config
    trajectoryConfig = new TrajectoryConfig(Constants.SPEC_MAX_SPEED, Constants.SPEC_MAX_ACCELERATION).setKinematics(driveKinematics).addConstraint(driveVoltageConstraint);

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

    // Puts the arcade drive boolean
    arcadeMode = false;
    SmartDashboard.putBoolean("Arcade Mode Enabled", arcadeMode);
    SmartDashboard.putBoolean("Curvature Mode Enabled", curvatureMode);

    // Sets up the drive ramping lastValue storage
    leftLastInput = 0.0;
    rightLastInput = 0.0;

    // Sets up the robot DifferentialDrive object for autonomous 
    robotDrive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);
  }

  public void initDefaultCommand() {
    // setDefaultCommand(new DriveTeleop());
  }

  @Override
  public void periodic() {
    // Updates the odometry object with new position info
    /* Need to update Encoders to output distance in meters instead of feet to work */
    driveOdometry.update(Rotation2d.fromDegrees(getMeasurement()), leftDriveEncoder.getPosition(),rightDriveEncoder.getPosition());

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Drive P Gain", 0);
    double i = SmartDashboard.getNumber("Drive I Gain", 0);
    double d = SmartDashboard.getNumber("Drive D Gain", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_leftDrive_pid.setP(p); m_rightDrive_pid.setP(p); kP = p; }
    if((i != kI)) { m_leftDrive_pid.setI(i); m_rightDrive_pid.setI(i); kI = i; }
    if((d != kD)) { m_leftDrive_pid.setD(d); m_rightDrive_pid.setD(d); kD = d; }

    // Enables Arcade Drive and Curvature Drive
    arcadeMode = SmartDashboard.getBoolean("Arcade Mode Enabled", false);
    curvatureMode = SmartDashboard.getBoolean("Curvature Mode Enabled", false);

    // Put motor encoder values on SmartDashboard for PID Graphing
    SmartDashboard.putNumber("Left Drive Speed",  leftDriveEncoder.getVelocity());
    SmartDashboard.putNumber("Right Drive Speed", rightDriveEncoder.getVelocity());

    // Put NavX Values on SmartDashboard
    SmartDashboard.putNumber("NavX Yaw", navX.getAngle());

    // Puts stick values on SmartDashboard
    SmartDashboard.putNumber("Left Stick Value", leftStickValue);
    SmartDashboard.putNumber("Right Stick Value", rightStickValue);
  }


  /* Teleop Methods */
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

  public void tankDrive(double leftValue, double rightValue, boolean squareInputs, boolean rampInputs) {
    // Replaces the regular tankdrive with a PID loop that controls the RPM
    // This compensates for the amperage drop in the battery and makes it much smoother

    // leftValue *= 0.5;
    // rightValue *= 0.5;
    if (squareInputs) 
    {
      // Squares the input to make it a exponential response curve instead of linear
      // to increase fine control while permitting full power
      leftValue = Math.copySign(squareInput(leftValue, 0.3), leftValue);
      rightValue = Math.copySign(squareInput(rightValue, 0.3), rightValue);
    }
    if (rampInputs)
    {
      // Ramp inputs to make acceleration much smoother
      leftValue = Math.copySign(rampInput(leftValue, Constants.MAX_ACCELERATION, leftLastInput, true), leftValue);
      rightValue = Math.copySign(rampInput(rightValue, Constants.MAX_ACCELERATION, rightLastInput, false), rightValue);
    }

    // Stores stick values into class variables
    leftStickValue = leftValue;
    rightStickValue = rightValue;
    
    robotDrive.tankDrive(leftValue, rightValue, false);
    
    // // Checks that the value is between -1 and 1
    // leftValue = MathUtil.clamp(leftValue, -1.0, 1.0);
    // rightValue = MathUtil.clamp(rightValue, -1.0, 1.0);
    
    // // Creates a deadzone on the controller to reduce drive jitter
    // leftValue = applyDeadband(leftValue, Constants.DEADBAND);
    // rightValue = applyDeadband(rightValue, Constants.DEADBAND);

    // // Square and Ramp Inputs 
    

    
    // // Converts the percentage value to RPM for the PID Loop
    // leftValue *= Constants.MAX_NEO_RPM;
    // rightValue *= Constants.MAX_NEO_RPM;

    // // Sets the reference point on the PID loop to the specified RPM
    // m_leftDrive_pid.setReference(leftValue, ControlType.kVelocity);
    // m_rightDrive_pid.setReference(rightValue, ControlType.kVelocity);

    // // Feeds the safety object with the new robot data
    // robotDrive.feed();
  }

  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs, boolean rampInputs)
  {
    if (squareInputs) 
    {
      // Squares the input to make it a exponential response curve instead of linear
      // to increase fine control while permitting full power
      xSpeed = Math.copySign(squareInput(xSpeed, 0.5), xSpeed);
      zRotation = Math.copySign(squareInput(zRotation, 0.5), zRotation);
    }
    if (rampInputs)
    {
      // Ramp inputs to make acceleration much smoother
      xSpeed = Math.copySign(rampInput(xSpeed, 1.0, leftLastInput, true), xSpeed);
      zRotation = Math.copySign(rampInput(zRotation, 1.0, rightLastInput, false), zRotation);
    }
    robotDrive.arcadeDrive(xSpeed, zRotation, true);
    // xSpeed *= 0.5;
    // zRotation *= 0.5;

    // xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    // xSpeed = applyDeadband(xSpeed, Constants.DEADBAND);

    // zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    // zRotation = applyDeadband(zRotation, Constants.DEADBAND);
    // zRotation *= -1;

    // // Speed Limiting
    // if(squareInputs)
    // {
    //   xSpeed = Math.copySign(squareInput(xSpeed, 0.7), xSpeed);
    //   zRotation = Math.copySign(squareInput(zRotation, 0.7), zRotation);
    // }
    // if(rampInputs)
    // {
    //   xSpeed = Math.copySign(rampInput(xSpeed, 1.0), xSpeed);
    //   zRotation = Math.copySign(rampInput(zRotation, 1.0), zRotation);
    // }

    // double leftMotorOutput;
    // double rightMotorOutput;

    // double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    // if (xSpeed >= 0.0) 
    // {
    //   // First quadrant, else second quadrant
    //   if (zRotation >= 0.0) 
    //   {
    //     leftMotorOutput = maxInput;
    //     rightMotorOutput = xSpeed - zRotation;
    //   } 
    //   else 
    //   {
    //     leftMotorOutput = xSpeed + zRotation;
    //     rightMotorOutput = maxInput;
    //   }
    // } 
    // else 
    // {
    //   // Third quadrant, else fourth quadrant
    //   if (zRotation >= 0.0) 
    //   {
    //     leftMotorOutput = xSpeed + zRotation;
    //     rightMotorOutput = maxInput;
    //   } 
    //   else 
    //   {
    //     leftMotorOutput = maxInput;
    //     rightMotorOutput = xSpeed - zRotation;
    //   }
    // }
    
    // leftMotorOutput = MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * Constants.MAX_NEO_RPM;
    // rightMotorOutput = MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * Constants.MAX_NEO_RPM;

    // m_leftDrive_pid.setReference(leftMotorOutput, ControlType.kVelocity);
    // m_rightDrive_pid.setReference(rightMotorOutput, ControlType.kVelocity);

    // robotDrive.feed();
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn)
  {
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    xSpeed = applyDeadband(xSpeed, Constants.DEADBAND);

    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    zRotation = applyDeadband(zRotation, Constants.DEADBAND);

    double angularPower;
    boolean overPower;

    double m_quickStopThreshold = Constants.QUICK_STOP_THRESHOLD;
    double m_quickStopAlpha = Constants.QUICK_STOP_ALPHA;
    double m_quickStopAccumulator = Constants.QUICK_STOP_ACCUMULATOR;

    if (isQuickTurn) 
    {
      if (Math.abs(xSpeed) < m_quickStopThreshold) 
      {
        m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
            + m_quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
      }
      overPower = true;
      angularPower = zRotation;
    } 
    else 
    {
      overPower = false;
      angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

      if (m_quickStopAccumulator > 1) 
      {
        m_quickStopAccumulator -= 1;
      } 
      else if (m_quickStopAccumulator < -1) 
      {
        m_quickStopAccumulator += 1;
      } 
      else 
      {
        m_quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = xSpeed + angularPower;
    double rightMotorOutput = xSpeed - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) 
    {
      if (leftMotorOutput > 1.0) 
      {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      }
      else if (rightMotorOutput > 1.0) 
      {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } 
      else if (leftMotorOutput < -1.0) 
      {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } 
      else if (rightMotorOutput < -1.0) 
      {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) 
    {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }

    leftMotorOutput *= Constants.MAX_NEO_RPM;
    rightMotorOutput *= Constants.MAX_NEO_RPM;

    m_leftDrive_pid.setReference(leftMotorOutput, ControlType.kVelocity);
    m_rightDrive_pid.setReference(rightMotorOutput, ControlType.kVelocity);

    robotDrive.feed();
  }

  public boolean arcadeModeOn() {
    // Returns whether or not arcade mode should be on
    return arcadeMode;
  }

  public boolean curvatureModeOn() {
    // Returns whether or not curvature mode should be on
    return curvatureMode;
  }
  
  double squareInput(double input, double degree) {
    // Adjustable parabolic curve for drive values
    return Math.pow(input, 3) + Constants.SQUARING_CONSTANT * (degree * input) / (Constants.SQUARING_CONSTANT * degree + 1);
  }

  double rampInput(double input, double maxAccel, double lastInput, boolean isLeft) {
    // Calculate the change from the current input from the last input
    double change = input - lastInput;
    // If the change is larger than max acceleration then ramp the acceleration down
    if (Math.abs(change) >= maxAccel) {
      change = Math.signum(change) * maxAccel;
    }
    // Add the change to the last input
    // Store the last input as this one
    // If/else statement decides if this change comes from the left stick or the right stick
    if(isLeft)
    {
      input = leftLastInput + change;
      leftLastInput = input;
    }
    else
    {
      input = rightLastInput + change;
      rightLastInput = input;
    }
    // Return the input
    return input;
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
    // Returns a Pose2D object from the odometry setup
    return driveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    driveOdometry.resetPosition(pose, Rotation2d.fromDegrees(navX.getYaw()));
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftDriveEncoder.getPosition(), rightDriveEncoder.getPosition());
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

  public double getRate(){
    // Returns the current rotation rate of the robot
    return navX.getRate();
  }

  public CANEncoder getLeftEncoder() {
    // Returns the left drive encoder
    return leftDriveEncoder;
  }

  public CANEncoder getRightEncoder() {
    // Returns the right drive encoder
    return rightDriveEncoder;
  }

  public void resetEncoders() {
    // Resets the encoder distance to 0
    leftDriveEncoder.setPosition(0.0);
    rightDriveEncoder.setPosition(0.0);
  }

  public double getAverageEncoderDistance() {
    return (leftDriveEncoder.getPosition() + rightDriveEncoder.getPosition()) / 2.0;
  }

  public boolean atSetPoint() {
    // Returns true if the PID loop is at its setpoint within the set tolerances
    return getController().atSetpoint();
  }


  /* Angle PID Methods */
    @Override
  public void useOutput(double output, double setpoint){
    rotateToAngleRate = output;
  }

    @Override
  public double getMeasurement() {
    return navX.pidGet();
  }
  
}
