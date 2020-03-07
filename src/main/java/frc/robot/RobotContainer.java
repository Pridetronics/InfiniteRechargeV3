/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DescendTelescopicClimb;
import frc.robot.commands.DriveJoystick;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.Drive;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.LowSpeedShooter;
import frc.robot.commands.RaiseRobot;
import frc.robot.commands.ReleaseGate;
import frc.robot.commands.RotateColorWheel;
import frc.robot.commands.ExtendRetractIntake;
import frc.robot.commands.ExtendTelescopicClimb;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.GoDistance;
import frc.robot.commands.GoToAngle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Scheduler;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton; //Deals with the buttons on the controller
import edu.wpi.first.wpilibj.Joystick; //Allows gamepad/joystick referencing

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer { // The robot's subsystems and commands are defined here...
  // The container for the robot. Contains subsystems, OI devices, and commands.
  public static CANSparkMax leftDriveMotorLead; // Creates new talon motor for leading left drive
  public static CANSparkMax rightDriveMotorLead; // Creates new talon motor for leading right drive
  public static CANPIDController leftDrive_pid;
  public static CANPIDController rightDrive_pid;
  public static CANEncoder leftDriveEncoder;
  public static CANEncoder rightDriveEncoder;

  public static Joystick joystickDriver; // The name of the first controller, main driver
  public static Joystick joystickShooter; // The name of the second controller, secondary driver

  public static CANSparkMax leftDriveMotorFollow;
  public static CANSparkMax rightDriveMotorFollow;
  public static TalonSRX intakeMotor;
  public static TalonSRX elevatorMotor;
  public static TalonSRX raiseRodMotor;
  public static CANSparkMax spoolWinchMotor;
  public static CANSparkMax shooterMotor;
  public static CANSparkMax colorWheelMotor;

  public JoystickButton intakeButton; // Button to run the intake
  public JoystickButton intakeExtendRetractButton; // Button to run the intake Vertical
  public JoystickButton lowSpeedShooterButton; // Button A
  public JoystickButton rotateColorWheelButton;

  public static Shooter shooter; // shooter object to be used for shooter commands
  public static CANPIDController shooterMotor_pid;

  public JoystickButton raiseTelescopicRodButton;
  public JoystickButton descendTelescopicRodButton;
  public JoystickButton winchMotorButton;

  public static DoubleSolenoid shooterBallRelease; // represents the solenoids for the different intake systems
  public static DoubleSolenoid intakeDeploy;
  public static DoubleSolenoid controlPanelSpinnerDeploy;
  public static DoubleSolenoid intakeExtendRetract;

  public static DigitalInput intakeLimitSwitch;
  public static DigitalInput shooterLimitSwitch;
  public static DigitalInput upperClimbLimitSwitch;
  public static DigitalInput lowerClimbLimitSwitch;

  public static CANEncoder shooterMotorEncoder; // encoder to measure the speed of the shooterMotor
  
  public final Drive robotDrive;
  
  public static Climb climb;

  public static Intake intake;

  public static ColorWheel colorWheel;

  // Counts how many balls are in the magazine
  public Counter ballCounter;

  private final SequentialCommandGroup m_auton;
  private final SequentialCommandGroup m_auton_traj;
  
  public RobotContainer() 
  {
    // Gets an instance of the command scheduler
    CommandScheduler scheduler = CommandScheduler.getInstance();

    /* Turn On LiveWindow */
    // Disables all motors and PID loop commands for testing in smartdashboard
    // https://first.wpi.edu/FRC/roborio/beta/docs/java/edu/wpi/first/wpilibj/livewindow/LiveWindow.html
    // LiveWindow.setEnabled(true);

    /********************************************************************************************/
    /*
        Joystick Controller
    */
    /********************************************************************************************/

    joystickDriver = new Joystick(Constants.DRIVER_JOYSTICK_NUMBER);
    joystickShooter = new Joystick(Constants.SHOOTER_JOYSTICK_NUMBER);
    
    /********************************************************************************************/
    /*
        Start of driver section
    */
    /********************************************************************************************/
    
    leftDriveMotorLead = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_LEAD, MotorType.kBrushless); // Creates new talon motor for leading left drive
    leftDriveMotorLead.setInverted(true); // Inverts Left Drive Motor
    leftDriveMotorLead.set(0); // Sets speed to 0 (anywhere between -1 and 1)

    leftDriveMotorFollow = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_FOLLOW, MotorType.kBrushless);
    leftDriveMotorFollow.follow(leftDriveMotorLead);
    
    rightDriveMotorLead = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_LEAD, MotorType.kBrushless); // Creates new talon motor for leading right drive
    rightDriveMotorLead.setInverted(false); // Inverts Right Drive Motor
    rightDriveMotorLead.set(0); // Sets speed to 0 (anywhere between -1 and 1)

    rightDriveMotorFollow = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_FOLLOW, MotorType.kBrushless);
    rightDriveMotorFollow.follow(rightDriveMotorLead);

    //Creates the encoders for the left drive and right drive motors
    leftDriveEncoder = new CANEncoder(leftDriveMotorLead, EncoderType.kHallSensor, 4096);
    rightDriveEncoder = new CANEncoder(rightDriveMotorLead, EncoderType.kHallSensor, 4096);
    
    /* Sets the gear ratio for the encoders to convert it to feet */
    /* Need to convert this to meters for odometry */
    leftDriveEncoder.setPositionConversionFactor(Constants.WHEEL_CIRCUMFERENCE / Constants.MAIN_MOTOR_RATIO); // Converts to distancse in feet and uses the gearbox ratio too
    rightDriveEncoder.setPositionConversionFactor(Constants.WHEEL_CIRCUMFERENCE / Constants.MAIN_MOTOR_RATIO); // Converts to distance in feet and uses the gearbox ratio too
    leftDriveEncoder.setInverted(true);
    rightDriveEncoder.setInverted(true);

    //Gets the pid controller for the left drive and right drive motors
    leftDrive_pid = leftDriveMotorLead.getPIDController();
    rightDrive_pid = rightDriveMotorLead.getPIDController();

    // Set the PID constants
    leftDrive_pid.setP(Constants.DRIVE_kP);
    leftDrive_pid.setI(Constants.DRIVE_kI);
    leftDrive_pid.setD(Constants.DRIVE_kD);

    rightDrive_pid.setP(Constants.DRIVE_kP);
    rightDrive_pid.setI(Constants.DRIVE_kI);
    rightDrive_pid.setD(Constants.DRIVE_kD);

    //Creates the drive object
    robotDrive = new Drive();

    /********************************************************************************************/
    /*  
        Start of Shooter section
    */
    /********************************************************************************************/

    //Creates shooter motor 
    shooterMotor = new CANSparkMax(Constants.SHOOTER_MOTOR_CAN_ADDRESS, MotorType.kBrushless);
    shooterMotor.setInverted(false);

    //Creates shooter motor pid controller and sets the pid constants
    shooterMotor_pid = shooterMotor.getPIDController();
    shooterMotor_pid.setP(Constants.SHOOTER_kP);
    shooterMotor_pid.setI(Constants.SHOOTER_kI);
    shooterMotor_pid.setD(Constants.SHOOTER_kD);
    shooterMotor_pid.setOutputRange(Constants.SHOOTER_MIN_OUTPUT, Constants.SHOOTER_MAX_OUTPUT);

    //Creates the encoder of the shooter motor
    shooterMotorEncoder = shooterMotor.getEncoder();

    //Creates the double solenoid for the shooter gate
    shooterBallRelease = new DoubleSolenoid(Constants.SHOOTER_GATE_FORWARD_CHANNEL, Constants.SHOOTER_GATE_RELEASE_CHANNEL);

    //Creates a shooter object
    shooter = new Shooter();
    
    /********************************************************************************************/
    /*  
        Start of intake section
    */
    /********************************************************************************************/
    
    //creates the double solenoid and gives it the forward/reverse channels
    intakeExtendRetract = new DoubleSolenoid(Constants.INTAKE_SOLENOID_FORWARD_CHANNEL, Constants.INTAKE_SOLENOID_REVERSE_CHANNEL);

    //creates the intake motor and sets the speed to 0
    intakeMotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR_CAN_ADDRESS);
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);

    /*
    //creates the elevator motor, sets the speed to 0, and has it follow the intake motor
    elevatorMotor = new WPI_TalonSRX(Constants.ELEVATOR_MOTOR_CAN_ADDRESS);
    elevatorMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    elevatorMotor.follow(intakeMotor);
    */

    //creates the intake object
    intake = new Intake();

    /********************************************************************************************/
    /*  
        Start of climb section
    */
    /********************************************************************************************/
    
    upperClimbLimitSwitch = new DigitalInput(Constants.UPPER_CLIMB_LIMIT_CHANNEL);
    lowerClimbLimitSwitch = new DigitalInput(Constants.LOWER_CLIMB_LIMIT_CHANNEL);
    // Creates the telescopic rod motor, sets it to 0, and sets it to brake mode
    raiseRodMotor = new WPI_TalonSRX(Constants.RAISE_CLIMB_MOTOR_ADDRESS);
    raiseRodMotor.setNeutralMode(NeutralMode.Brake);
    raiseRodMotor.set(TalonSRXControlMode.PercentOutput, 0.0);

    // Creates the motor to spool the winch, sets it to brake mode, and sets it 0
    spoolWinchMotor = new CANSparkMax(Constants.TELESCOPIC_CLIMB_MOTOR_ADDRESS, MotorType.kBrushless);
    spoolWinchMotor.setIdleMode(IdleMode.kBrake);
    spoolWinchMotor.setInverted(true);
    spoolWinchMotor.set(0.0);

    // Creates climb object
    climb = new Climb();

    /*********************************************************************************************/
    /*
        Start of color wheel section
    */
    /*********************************************************************************************/
    /*
    //Creates the motor and sets it to 0
    colorWheelMotor = new CANSparkMax(Constants.COLOR_WHEEL_MOTOR_ADDRESS, MotorType.kBrushless);
    colorWheelMotor.set(0);

    //Creates new ColorWheel object
    colorWheel = new ColorWheel();

    //Command to rotate the color wheel a certain number of times
    rotateColorWheelButton.whenPressed(new RotateColorWheel(colorWheel));
    */
    /*********************************************************************************************/
    /*
        Start of miscellaneous section
    */
    /*********************************************************************************************/

    //new InstantCommand(shooter, shooter::releaseGate());
    
    // Extends the shooter ball blocker on start of robot
    scheduler.schedule(new InstantCommand(shooter::retractGate, shooter));
    /*
    intakeLimitSwitch = new DigitalInput(Constants.INTAKE_LIMIT_SWITCH_CHANNEL);
    shooterLimitSwitch = new DigitalInput(Constants.SHOOTER_LIMIT_SWITCH_CHANNEL);

    ballCounter = new Counter(CounterBase.EncodingType.k2X, intakeLimitSwitch, shooterLimitSwitch, false);
    */

   //new InstantCommand(Shooter::releaseGate, shooter);
  
    //Test autonomous command to test GoDistance and GoToAngle commands
    m_auton = new SequentialCommandGroup(
        new GoDistance(4.0, robotDrive),
        new GoDistance(-4.0, robotDrive)
    );

    // Trajectory Autonomous Command
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        robotDrive.trajectoryConfig
    );
    m_auton_traj = new SequentialCommandGroup(new FollowTrajectory(robotDrive, trajectory));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    /* Sets up the buttons on the joystick */
    lowSpeedShooterButton = new JoystickButton(joystickShooter, Constants.SHOOTER_BUTTON_NUMBER); // creates the button for the low speed shooter
    intakeButton = new JoystickButton(joystickDriver, Constants.INTAKE_BUTTON_NUMBER); // Right Upper Bumper, sets intake Button to a controller
    raiseTelescopicRodButton = new JoystickButton(joystickShooter, Constants.TELESCOPIC_ROD_BUTTON_NUMBER);
    descendTelescopicRodButton = new JoystickButton(joystickShooter, Constants.DESCEND_ROBOT_BUTTON_NUMBER);
    winchMotorButton = new JoystickButton(joystickShooter, Constants.WINCH_BUTTON_NUMBER);
    rotateColorWheelButton = new JoystickButton(joystickShooter, Constants.ROTATE_COLOR_WHEEL_BUTTON);

    /* Binds the buttons to each command  */
    // Calls the command to run the shooter motor and release the shooter gate at the same time

    lowSpeedShooterButton.whenHeld(new ParallelCommandGroup(
        new LowSpeedShooter(shooter),
        new WaitCommand(0.2),
        new ReleaseGate(shooter)));

    // Runs the command to extend the intake and run the intake/elevator motors at the same time
    intakeButton.whenHeld(new ParallelCommandGroup(
        new ExtendRetractIntake(intake),
        new WaitCommand(0.3),
        new IntakeRun(intake)));

    // Sets up the Drive commands
    robotDrive.setDefaultCommand(new DriveJoystick(joystickDriver, robotDrive));
    
    // Command to raise the telescopic rod
    raiseTelescopicRodButton.whenHeld(new ExtendTelescopicClimb(climb));

    //Command to bring the telescopic rod down and spool the winch to raise the robot
    descendTelescopicRodButton.whenHeld(new DescendTelescopicClimb(climb));
       
    winchMotorButton.whenHeld(new RaiseRobot(climb));
  }


  /*
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_auton;
  }
}