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

/*
  To-do:
  The double solenoids are only initialized and need to be instantiated. To do that, you need the forward and
  reverse channels. I will find those out on monday. For now, I will declare them as constants in Constants.
*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.button.JoystickButton; //Deals with the buttons on the controller
import edu.wpi.first.wpilibj.Joystick; //Allows gamepad/joystick referencing

//import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer { // The robot's subsystems and commands are defined here...
  // The container for the robot.  Contains subsystems, OI devices, and commands.
  public static CANSparkMax leftDriveMotorLead; // Creates new talon motor for leading left drive
  public static CANSparkMax rightDriveMotorLead; // Creates new talon motor for leading right drive
  
  public static Joystick joystickDriver; //The name of the first controller, main driver
  public static Joystick joystickShooter; //The name of the second controller, secondary driver
  
  public JoystickButton intakeButton; //Button to run the intake 
  public JoystickButton intakeExtendRetractButton; //Button to run the intake Vertical
  public static CANSparkMax intakeMotor;
  public static CANSparkMax elevatorMotor;
  public static TalonSRX raiseRodMotor;
  public static CANSparkMax spoolWinchMotor;
  public final Drive robotDrive;

  public JoystickButton lowSpeedShooterButton; // Button A
  public JoystickButton highSpeedShooterButton; // Button Y
  public JoystickButton cameraModeButton;

  public static Shooter shooter; // shooter object to be used for shooter commands

  public static CANSparkMax shooterMotor;

  //public Pneumatics pneumatics; // creates a pneumatic object
  
  public static DoubleSolenoid shooterBallRelease; // represents the solenoids for the different intake systems
  public static DoubleSolenoid intakeDeploy;
  public static DoubleSolenoid controlPanelSpinnerDeploy;

  public static CANEncoder shooterMotorEncoder; // encoder to measure the speed of the shooterMotor

  public JoystickButton raiseTelescopicRodButton;
  public JoystickButton liftRobotButton;

  public static Climb climb;
  
  public DigitalInput intakeLimitSwitch;
  public DigitalInput shooterLimitSwitch;
  public static DigitalInput upperClimbLimitSwitch;
  public static DigitalInput lowerClimbLimitSwitch;

  public static DoubleSolenoid intakeExtendRetract;

  public static Intake intake;

  public BaseMotorController talonMotorController;

  public Counter ballCounter;

  public RobotContainer() {
    
    //Joystick+Controller Definitions 
    this.joystickDriver = new Joystick(0); // 'this.' Grabs a variable specifically
    this.joystickShooter = new Joystick(1); // ^^ Creates less confusion in the system
    // The numbers in the parenthesis represents the ports each controller goes to. 
    
    /*
      Start of driver section
    */
    
    leftDriveMotorLead = new CANSparkMax(Constants.leftDriveMotorLead, MotorType.kBrushed); // Creates new talon motor for leading left drive
    leftDriveMotorLead.setInverted(false); // Inverts Left Drive Motor
    leftDriveMotorLead.set(0); // Sets speed to 0 (anywhere between -1 and 1)
    
    rightDriveMotorLead = new CANSparkMax(Constants.rightDriveMotorLead, MotorType.kBrushed); // Creates new talon motor for leading right drive
    rightDriveMotorLead.setInverted(false); // Inverts Right Drive Motor
    rightDriveMotorLead.set(0); // Sets speed to 0 (anywhere between -1 and 1)

    robotDrive = new Drive(); 
    // It sets a new drive and uses the ints 1 and 2. The order matters.
    // 1 is assigned to leftDriveMotorAddress, whereas 2 is rightDriveMotorAddress

    robotDrive.setDefaultCommand(new DriveJoystick(joystickDriver, robotDrive));
    // This helps set the default command. It sets it to DriveJoystick so that way RobotContainer
    // can grab the information and utilize it for the given controller, in this case joystickDriver
    
    /*
    Start of Shooter section
    */
    
    shooterMotor = new CANSparkMax(Constants.shooterMotorCanAddress, MotorType.kBrushed); // instantiates new shooter motor with specific ID

    shooterMotorEncoder = new CANEncoder(shooterMotor, EncoderType.kHallSensor, 42); // instantiates a new encoder for the shooterMotor
    
    shooterBallRelease = new DoubleSolenoid(Constants.shooterGateForwardChannel, Constants.shooterGateReverseChannel);
    
    shooter = new Shooter(); // new Shooter object
    
    //pneumatics = new Pneumatics(); //instantiates a new pneuamtics object

    
    
    
    lowSpeedShooterButton = new JoystickButton(this.joystickShooter, 1); // creates the button for the low speed shooter
    
    /*
      I'm using a command group to run through the shooter code. Since a command group is recognized as a
      command, you can use it another command group. I use a sequential command group, which runs the commands
      sequentially. The first command it runs is the ParallelDeadlineGroup, which allows me to run multiple
      commands at the same time. With this command, you can decide the deadline. The deadline is the command
      that decides when the command group will end. In this case, LowSpeedShooter is the deadline, so when
      that command ends, the whole command group ends. In the ParallelDeadlineGroup, the shooter command
      and the ReleaseGate command run together. After this command is done, the CloseGate command is run,
      and when that finishes, the SequentialCommandGroup command ends.
    */
    
    
    lowSpeedShooterButton.whenHeld(new ParallelCommandGroup(
        new LowSpeedShooter(this.joystickShooter, shooter),
        new ReleaseGate(this.joystickShooter, shooter, Constants.lowShooterSpeed)));
      
    /*
      The whenHeld method runs the low speed shooter command when the A button is held.
      The method requires an object of a command, such as new LowSpeedShooter
    */
    
    /*
      see the comment above lowSpeedShooterButton.whenHeld for an explanation
    */
    
        
    /*
      The whenHeld method runs the high speed shooter command when the Y button is held.
      The method requires an object of a command, such as new HighSpeedShooter
    */

    //Motor Definitions

    /*
      Start of intake section
    */
    
    intakeExtendRetract = new DoubleSolenoid(Constants.INTAKE_SOLENOID_FORWARD_CHANNEL, Constants.INTAKE_SOLENOID_REVERSE_CHANNEL);
    
    intakeMotor = new CANSparkMax(Constants.intakeMotorCanAddress, MotorType.kBrushless); //The motor (CANSparkMax) is defined with a type and port (port 5, and motor type = brushless)
    ///intakeMotor =  new Talon(5); //Motor is defined as a specified motor under port five (Talon)
    intakeMotor.set(0); //Initially sets motor value to 0, will not run without further command

    elevatorMotor = new CANSparkMax(Constants.elevatorMotorFollowAddress, MotorType.kBrushless); //Motor is deinfed under 7th port (CANSparkMax)
    ///elevatorMotorFollow = new Talon(7); // Motor is defined under port seven (Talon)
    elevatorMotor.set(0); //Sets motor speed to 0
    elevatorMotor.follow(intakeMotor); // Vertical follow motor will do everthing the vertical lead motor does

    intakeButton = new JoystickButton(joystickDriver, 5); // Right Upper Bumper, sets intake Button to a controller
    intakeExtendRetractButton = new JoystickButton(joystickDriver, 7); //Left Upper Bumper, elevator button  to a controller
    
    intake = new Intake();

    intakeExtendRetractButton.whenPressed(new ExtendRetractIntake(intake));//While held, command is being run, references command from commands. Hence imports
    intakeButton.whenHeld(new IntakeRun(intake));//While the button is being held, the command is being run
    
    /*
      Start of climb section
    */

    upperClimbLimitSwitch = new DigitalInput(Constants.upperClimbLimitChannel);
    lowerClimbLimitSwitch = new DigitalInput(Constants.lowerClimbLimitChannel);
    /*
      If the limit switch is closed, the value is 0. If the limit switch is open, the value is 1
    */
    
    raiseRodMotor = new WPI_TalonSRX(Constants.raiseClimbMotorAddress);
    raiseRodMotor.setNeutralMode(NeutralMode.Brake);
    //talonMotorController = new BaseMotor(Constants.raiseClimbMotorAddress, );
    //raiseClimbMotor.set(0);

    spoolWinchMotor = new CANSparkMax(Constants.telescopicClimbMotorAddress, MotorType.kBrushless);
    spoolWinchMotor.setIdleMode(IdleMode.kBrake);
    //telescopicClimbMotor.set(0);
    // The numbers in the parenthesis represents the ports each controller goes to. 

    raiseTelescopicRodButton = new JoystickButton(joystickShooter, 6);
    liftRobotButton = new JoystickButton(joystickShooter, 5);

    climb = new Climb();

    raiseTelescopicRodButton.whileHeld(new ExtendTelescopicClimb(joystickShooter, climb));

    /*
    liftRobotButton.whileHeld(new SequentialCommandGroup(
        new DescendTelescopicClimb(joystickShooter, climb),
        new RaiseRobot(Constants.WINCH_TIMEOUT, climb)));
    */
    intakeLimitSwitch = new DigitalInput(Constants.intakeLimitSwitchChannel);
    shooterLimitSwitch = new DigitalInput(Constants.shooterLimitSwitchChannel);

    ballCounter = new Counter(CounterBase.EncodingType.k2X, intakeLimitSwitch, shooterLimitSwitch, false);
    

    // Configure the button bindings
  
    
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // final JoystickButton switchDriveMode = new JoystickButton(null, 8);

  }


  /*
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return getAutonomousCommand();
  }
}