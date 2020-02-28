/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
  ddddddddddddddddddddddddddddddddddddddddddddooooolllcc::;;;;;::ccllloooooooooooooooooooooooooooooooollllllllll
ddddddddddddddddddddddddddddddddddooooooooollccc::;;::::cclloooddddddddddddddddddddddddddddddoooooooooooooooll
dddddddddddddddddddddddddddddoooooooolllcc:::::::clloodddddxxxxxxxxxxxxxxxxxxxxxxxxxxddddddddddddddooooooooooo
dddddddddddddddddddddoooooooooolllcc::::::ccloodddxxxxxxxkkkkkkkkkkkkkkkkkkkkkkkxxxxxxxxxxxxdddddddddooooooooo
oooddoooooooooodoooooooolllcc:::::::cllooddxxxxkkkkkkkkkkkkkkOOOOOOOOOkkkkkkkkkkkkkkxxxxxxxxxxxxdddddddoddoooo
oooooooooooooooooolllcc:::;::cclooddxxxxkkkkkkkOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOkkkkkkkkkkkxxxxxxxxxddddddddddooo
oooooooooolllllcc::::::ccloodddxxxkkkkkkOOOOOOOOOO000000000000000000000O00OOOOOOOkkkkkkkkkkkxxxxxxxxdddddddddo
lllllllccc:::;;;::cllodddxxxkkkkOOOOOOOO0000000000000000000K00K0000000000000OOOOOOOOOOkkkkkkkkxxxxxxxxdddddddd
ccc::::;;;::cclooddxxxkkkkkOOOOOOO0000000000KKKKKKKKKKKKKKKKKKKKKKKKKK0000K0O000OOOOOkkkkkkkkkxxxxxxxxxxxddddd
;;;;;:cclloodddxxxkkkkkOOOOO00000000KKKKKKKKKKKKKKKKKKKKKXKKKKKKKKKKKKK00K0OOOxxxkOkdddxkkkkkkkkxxxxxxxxxxdddd
:cclloodddxxxxkkkkOOOOO00000000KKKKKKKKKKKKKKXXXXXXXXXXXKKKKKO0K0O0KKKK00OkOOkocldxo:ollkkkkkkkkkkkxxxxxxxxddd
ooodddxxxxkkkkkkOOOOO000000KKKKKKKKKKXXXXXXXXXXXXXXXXXKKKX0OkdoxdodO00000Odloolc:cc;;c:lxdodkkkkkkkkxxxxxxxxdd
ddddxxxkkkkkOOOO0000000KKKKKKKXXKXXXXXXXXXXXXNNNXNNXXXK00OOkdl:::;;;:cldddoc:;;;,,'',;;::;:oxkkkkkkkkkxxxxxxxx
dxxxkkkkkOOOOO0000000KKKKKKKXXXXXXXXXXXXNNNNNNNNNNNNNXKOxlloddoc;'....,::;::'',,;;'',,',,';dOkkkkkkkkkxxxxxxxx
xxkkkkOOOOOO00000KKKKKKKKKXXXXXXXXNNNNNNNNNNNNNNNNNNNXOddc;cxxoccoc,'''..','.....,,'....'..cddxkkkkkkkkxxxxxxx
kkkkOOOOOO00000KKKKKKKXXXXXXXXNNNNNNNNNNNNNNNNNNNNNNNKdcclokxdoc::;,''..'..................',cdxxxkkxxkkxxxxxx
kkOOOOO0000000KKKKKKKKXXXXXXNNNNNNNNNNNNNNNNNNNNNNWN0xdxddxxoooc:;,'''........................,:ldxxkkkkkxxxxx
kOOOOO0000000KKKKKKKKXXXXXXXNNNNNNNNNNNNNNNNNNNNWWN0xxxxollooddolc;,,,'''''',,;;,,,'............,cxkkkkkkkxxxx
OOOOOO000000KKKKKKKKXXXXXXXNNNNNNNNNNNNNNNNWWWWWNNKkkxoolcldxkkxdolccc::::::cllllolc:;,''........,okkkkkkkkxxx
OOOOO0000000KKKKKKKXXXXXXXXNNNNNNNNNNNWNWWNWWWNWN0kO0kxkdodxxxxdddddooollllllooooddoolc:;;,'......,dkkkkkkkxxx
OOOO0000000KKKKKKKKXXXXXXXNNNNNNNNNNNNWNWWNWWNWNOdx0000000OOkxxxxxxxdddddooooooddooooolllcc:,......:xkkkkkkxxx
OOO00000000KKKKKKKKXXXXXXXXNNNNNNNNNNNNNWNNWNNW0ookKXNXKXXK0Okxxxxxxddddddoooooodooooooollllc;'....,dkkkkkkxxx
OOO00000000KKKKKKKKKXXXXXXXXNNNNNNNNNNNNNNNNNWNkldk0XNXXXXK0Okkxxxxxxxddddddoooooooooooollllll:,....cxkkkkkxxx
OOOO0000000KKKKKKKKKKXXXXXXXXXNNNNNNNNNNNNNNNNKoloxOKKXNXXXXK0Okkxxxxxddddddddddddooooooloolllc:,...,dkkkkkxxx
OOOO00000000KKKKKKKKKKKXXXXXXXXXXXNNNNNNNNNNNNkc:coxOKNNNNNX0Okxxxddddooddooooodooooodoooooololc;'..'lkkkkkkxx
OOOO0000000000KKKKKKKKKKXXXXXXXXXXXXXXNNNXNNNXd;,,cx0XNNXX0Oxdollccccclloollloolllllooooooooooll:,'..:xkkkkkxx
OOOO00000000000KKKKKKKKKKKKXXXXXXXXXXXXXXXXXNKo,.,lk00kxdlc;,,''...'',;:clllcc::;,,,;;::ccllllllc,'..,dkkkkxxx
OOOO000000000000KKKKKKKKKKKKKXXXXXXXXXXXXXXXX0c'.:k0kl;,,'...........'',:cllc;,'.........'',;::cc;...;xkkkkxxx
OOOOOOO000000000000KKKKKKKKKKKKKKKXXXXXXXXXXXk:';d0Oo;'''''...........';loddl:;,''...........'',;;...cxkkkkxxx
OOOOOOOOO0000000000000KKKKKKKKKKKKKKKKKKKKKXKd;,l00o:,''',,'. ........,cdxxdoc:,''......'''''',,;:'..lkkkkkxxx
OOOOOOOOOOO00000000000000000KKKKKKKKKKKKKKKXKd;ckXOocc:::::,.........,:oxkxdoc:,'.........,,'',;:c,..lkkkkxxxx
OOOOOOOOOOOO00000000000000000KKKKKKKKKKKKKK0kocxXXK0Okdddoc:;;,,,,;;:lox0Kkdol:;,'....   ..,,,,,;c;..lkkkkxxxx
OOOOOOOOOOOOOOOOOOOOOOOOOOOOO00000000000000o::l0XXXXK0OkxdoolllllloooookXXOdlcc:;;,''....',;;;:::cc..lkkkxxxxx
kkkxxxxxxxxdxxxxddddddddxxdddddddddddddddxo:cld0XKKKKK00Okkxddddddoooox0NKkdolccccc::;;;;;::ccccclc,.:xkxxxxxx
dooooooooooooooooooooooddddddddddoooooodddolood00OOkkOOOOOkkxxddddooookKK0xdoolcccclllllcccccccllll:';lxxxxxxx
lllloooooollllllooooooooooooooooooooooooooodocoOOkxxkkOOOOkkxddoooooodO0Okdoooocccccllllllollllllllc,',lxxxxxx
;:ccclllllllllllllllllllllooooloooolllllllokxodkxxxxkkxxxxdddoolllloxOOkkdooolllc::cclllllooollllllc;,;lxxxxxx
,,,;;;::ccccccllllllllllllooollllloolllllllxO00kxddxxdoooooooolllclxOkoccc:ccccc:::::clllllllllccccc;:cdxxxxxx
.''',,,,,,;;;;;;;;;::::::::ccccclllccc:;'',o00Okxddddooooooollccllooc;.','',;;,;::;;::ccllllllccccc:::ldxxxxxx
.......'''''',,,,,,,,,,,,,,,,,,,,,,,,,,'...:xkkkxdddddddooolcccclooc,....'''''..,;;;;;:cccccccccccc;;coxxxxxxx
..................'''''''''''''......''''..';:;lxdooddoooollccloodddo:;,'.''',',;:::;;;:::ccccccccc::ldxxxxxxx
...............................................ckdooddoooollooooddddolc:;,'',;;;:::::;;;;:::::ccc:cccoxxxxxxxx
          .....................................:xxddddddooodddddddolc:::;,',,;;;;:::::::::::::::c::cldxxxxxxxx
            .................................  ;xxdddddddodddollllcccccc:;;;;;;;;;;;:::::::::::::ccldxxxxxxxxd
                    ...............            'dxddddddoollc;;;;::cccccccclc::::::::;;::::::::::clodxxxxxxddd
                      ....                     .ldodddooolc;,''''''''''.'',,,''''''',,,;::::::::codddxxxxxdddd
                                               .:doooolllc:;;;:c::::;;;;,'''''','''''',;;:::::::ldxxxxxdxddddd
                                                ;olooolccc::::cccccccccc::::;;;;;;;::;;;;:c:::::oxxxxxdxxddddd
                                               .:dlllllccc:::cccccc:::;,,'''',,;;;::::;;;;:::::cdxxxxddddddddd
                                               .lxlcllclcc::::c::::;;,,''''''',,;;;;;;;;;;:::::odxxxdddddddddd
                                               .okocccclcccc:::::::;;;;;,,,,,,;;,;;;;;;;;;;;;:lddddddddddddddd
                                               ,xkxoccccclllcccc:::::ccccccc:::;;;;;;;;;;;;,;cdxdddddddddddddd
                                               :kxxdlc::cllllllccccclllllllcccc::::::;;;;;;;;ldddddddddddddddd
                                              .okxxxol:;:cllllllllllllllccccccc::::::;;;;,,;:odddddddddddddooo
                                              'xkxxddoc;;:ccllllllllllcc:::::::::::::;;;;,,;:oddddddddddoooooo
*/

package frc.robot;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DescendTelescopicClimb;
import frc.robot.commands.DriveJoystick;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.LowSpeedShooter;
import frc.robot.commands.RaiseRobot;
import frc.robot.commands.ReleaseGate;
import frc.robot.commands.ExtendRetractIntake;
import frc.robot.commands.ExtendTelescopicClimb;
import frc.robot.commands.GoDistance;
import frc.robot.commands.GoToAngle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
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
  public static CANSparkMax intakeMotor;
  public static CANSparkMax elevatorMotor;
  public static TalonSRX raiseRodMotor;
  public static CANSparkMax spoolWinchMotor;
  public static CANSparkMax shooterMotor;

  public JoystickButton intakeButton; // Button to run the intake
  public JoystickButton intakeExtendRetractButton; // Button to run the intake Vertical
  public JoystickButton lowSpeedShooterButton; // Button A
  public JoystickButton highSpeedShooterButton; // Button Y
  public JoystickButton cameraModeButton;

  public static Shooter shooter; // shooter object to be used for shooter commands
  public static CANPIDController shooterMotor_pid;

  // public Pneumatics pneumatics; // creates a pneumatic object
  public JoystickButton raiseTelescopicRodButton;
  public JoystickButton liftRobotButton;

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

  // Counts how many balls are in the magazine
  public Counter ballCounter;

  private final SequentialCommandGroup m_auton;
  
  public RobotContainer() 
  {

    /********************************************************************************************/
    /*
        Joystick Controller and Button Definitions
    */
    /********************************************************************************************/

    joystickDriver = new Joystick(0);
    joystickShooter = new Joystick(1);
    // The numbers in the parenthesis represents the ports each controller goes to.

    lowSpeedShooterButton = new JoystickButton(joystickShooter, 1); // creates the button for the low speed shooter
    intakeButton = new JoystickButton(joystickDriver, 5); // Right Upper Bumper, sets intake Button to a controller
    intakeExtendRetractButton = new JoystickButton(joystickDriver, 7); //Left Upper Bumper, elevator button  to a controller
    raiseTelescopicRodButton = new JoystickButton(joystickShooter, 6);
    liftRobotButton = new JoystickButton(joystickShooter, 5);

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

    leftDriveEncoder = new CANEncoder(leftDriveMotorLead, EncoderType.kHallSensor, 4096);
    rightDriveEncoder = new CANEncoder(rightDriveMotorLead, EncoderType.kHallSensor, 4096);
    
    /*
    leftDriveEncoder = leftDriveMotorLead.getEncoder();
    rightDriveEncoder = rightDriveMotorLead.getEncoder();
    */

    /* Sets the gear ratio for the encoders to convert it to feet */
    /* Need to convert this to meters for odometry */
    leftDriveEncoder.setPositionConversionFactor(Constants.WHEEL_CIRCUMFERENCE / Constants.MAIN_MOTOR_RATIO); // Converts to distance in feet and uses the gearbox ratio too
    rightDriveEncoder.setPositionConversionFactor(Constants.WHEEL_CIRCUMFERENCE / Constants.MAIN_MOTOR_RATIO); // Converts to distance in feet and uses the gearbox ratio too

    leftDrive_pid = leftDriveMotorLead.getPIDController();
    rightDrive_pid = rightDriveMotorLead.getPIDController();

    // Set the PID constants
    leftDrive_pid.setP(Constants.DRIVE_kP);
    leftDrive_pid.setI(Constants.DRIVE_kI);
    leftDrive_pid.setD(Constants.DRIVE_kD);

    rightDrive_pid.setP(Constants.DRIVE_kP);
    rightDrive_pid.setI(Constants.DRIVE_kI);
    rightDrive_pid.setD(Constants.DRIVE_kD);

    robotDrive = new Drive();
    // It sets a new drive and uses the ints 1 and 2. The order matters.
    // 1 is assigned to leftDriveMotorAddress, whereas 2 is rightDriveMotorAddress

    robotDrive.setDefaultCommand(new DriveJoystick(joystickDriver, robotDrive));
    // This helps set the default command. It sets it to DriveJoystick so that way
    // RobotContainer
    // can grab the information and utilize it for the given controller, in this
    // case joystickDriver

    /*
    JoystickButton testButton = new JoystickButton(joystickDriver, 3);
    testButton.whenPressed(new GoDistance(4.0, robotDrive));

    JoystickButton testButton2 = new JoystickButton(joystickDriver, 2);
    testButton2.whenPressed(new GoToAngle(90.0, robotDrive));
    */

    /********************************************************************************************/
    /*  
        Start of Shooter section
    */
    /********************************************************************************************/

    shooterMotor = new CANSparkMax(Constants.SHOOTER_MOTOR_CAN_ADDRESS, MotorType.kBrushless); // instantiates new shooter
                                                                                          // motor with specific ID
    // Shooter PID Setup
    shooterMotor_pid = shooterMotor.getPIDController();
    shooterMotor_pid.setP(Constants.SHOOTER_kP);
    shooterMotor_pid.setI(Constants.SHOOTER_kI);
    shooterMotor_pid.setD(Constants.SHOOTER_kD);

    shooterMotorEncoder = new CANEncoder(shooterMotor, EncoderType.kHallSensor, 4096); // instantiates a new encoder for
                                                                                     // the shooterMotor
    shooterBallRelease = new DoubleSolenoid(Constants.SHOOTER_GATE_FORWARD_CHANNEL, Constants.SHOOTER_GATE_RELEASE_CHANNEL);

    shooter = new Shooter(); // new Shooter object
    
    lowSpeedShooterButton.whenHeld(new ParallelCommandGroup(
        new LowSpeedShooter(shooter),
        new ReleaseGate(shooter)));
    
    /********************************************************************************************/
    /*  
        Start of intake section
    */
    /********************************************************************************************/
    
    intakeExtendRetract = new DoubleSolenoid(Constants.INTAKE_SOLENOID_FORWARD_CHANNEL, Constants.INTAKE_SOLENOID_REVERSE_CHANNEL);
    
    intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ADDRESS, MotorType.kBrushless); //The motor (CANSparkMax) is defined with a type and port (port 5, and motor type = brushless)
    ///intakeMotor =  new Talon(5); //Motor is defined as a specified motor under port five (Talon)
    intakeMotor.set(0); //Initially sets motor value to 0, will not run without further command

    elevatorMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR_CAN_ADDRESS, MotorType.kBrushless); //Motor is deinfed under 7th port (CANSparkMax)
    ///elevatorMotorFollow = new Talon(7); // Motor is defined under port seven (Talon)
    elevatorMotor.set(0); //Sets motor speed to 0
    elevatorMotor.follow(intakeMotor); // Vertical follow motor will do everthing the vertical lead motor does
    
    intake = new Intake();

    intakeExtendRetractButton.whenPressed(new ExtendRetractIntake(intake));//While held, command is being run, references command from commands. Hence imports
    intakeButton.whenHeld(new IntakeRun(intake));//While the button is being held, the command is being run
    
    /********************************************************************************************/
    /*  
        Start of climb section
    */
    /********************************************************************************************/

    /*
    upperClimbLimitSwitch = new DigitalInput(Constants.UPPER_CLIMB_LIMIT_CHANNEL);
    lowerClimbLimitSwitch = new DigitalInput(Constants.LOWER_CLIMB_LIMIT_CHANNEL);
    */
    /*
      If the limit switch is closed, the value is 0. If the limit switch is open, the value is 1
    */
    
    raiseRodMotor = new WPI_TalonSRX(Constants.RAISE_CLIMB_MOTOR_ADDRESS);
    raiseRodMotor.setNeutralMode(NeutralMode.Brake);

    spoolWinchMotor = new CANSparkMax(Constants.TELESCOPIC_CLIMB_MOTOR_ADDRESS, MotorType.kBrushless);
    spoolWinchMotor.setIdleMode(IdleMode.kBrake);
    // The numbers in the parenthesis represents the ports each controller goes to. 

    climb = new Climb();

    raiseTelescopicRodButton.whileHeld(new ExtendTelescopicClimb(climb));

    /*
    liftRobotButton.whileHeld(new SequentialCommandGroup(
        new DescendTelescopicClimb(climb),
        new RaiseRobot(Constants.WINCH_TIMEOUT, climb)));
    */
    
    /*********************************************************************************************/
    /*
        Start of miscellaneous section
    */
    /*********************************************************************************************/
    
    /*
    intakeLimitSwitch = new DigitalInput(Constants.INTAKE_LIMIT_SWITCH_CHANNEL);
    shooterLimitSwitch = new DigitalInput(Constants.SHOOTER_LIMIT_SWITCH_CHANNEL);

    ballCounter = new Counter(CounterBase.EncodingType.k2X, intakeLimitSwitch, shooterLimitSwitch, false);
    */
  
    m_auton = new SequentialCommandGroup(
              new GoDistance(4.0, robotDrive), new GoToAngle(90.0, robotDrive),
              new GoDistance(4.0, robotDrive), new GoToAngle(90.0, robotDrive),
              new GoDistance(4.0, robotDrive), new GoToAngle(90.0, robotDrive),
              new GoDistance(4.0, robotDrive), new GoToAngle(90.0, robotDrive));

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