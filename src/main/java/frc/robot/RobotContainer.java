/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CloseGate;
import frc.robot.commands.DriveJoystick;
import frc.robot.subsystems.Drive;
//import frc.robot.commands.IntakeRun;
import frc.robot.commands.LowSpeedShooter;
import frc.robot.commands.ReleaseGate;
import frc.robot.commands.ElevatorRun;
import frc.robot.commands.HighSpeedShooter;
import frc.robot.commands.IntakePneumaticExtend;
import frc.robot.commands.IntakePneumaticRetract;
//import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Elevator;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import frc.robot.commands.DriveForwardThreeFeetAuton;


import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.button.JoystickButton; //Deals with the buttons on the controller
import edu.wpi.first.wpilibj.Joystick; //Allows gamepad/joystick referencing
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public static CANSparkMax leftDriveMotorFollow;
  public static CANSparkMax rightDriveMotorFollow;
  
public static CANEncoder leftDriveMotorLeadEncoder;
public static CANEncoder rightDriveMotorLeadEncoder;

  public Joystick joystickDriver; //The name of the first controller, main driver
  public Joystick joystickShooter; //The name of the second controller, secondary driver
  
  public JoystickButton intakeButton; //Button to run the intake 
  public JoystickButton elevatorButton; //Button to run the intake Vertical
  public JoystickButton intakePneumaticExtendButton;
  public JoystickButton intakePneumaticRetractButton;
  public JoystickButton autonDriveForw;
  //public static CANSparkMax intakeMotor;
  public static CANSparkMax elevatorMotorLead;
  public static CANSparkMax elevatorMotorFollow;
  public static CANSparkMax raiseClimbMotor;
  public static CANSparkMax telescopicClimbMotor;
  public final Drive robotDrive;

  public JoystickButton lowSpeedShooterButton; // Button A
  public JoystickButton highSpeedShooterButton; // Button Y

  public Shooter shooter; // shooter object to be used for shooter commands

  public static CANSparkMax shooterMotor;

  public Pneumatics pneumatics; // creates a pneumatic object
  
  public static DoubleSolenoid shooterBallRelease; // represents the solenoids for the different intake systems
  public static DoubleSolenoid intakeDeploy;
  public static DoubleSolenoid controlPanelSpinnerDeploy;

  public static CANEncoder shooterMotorEncoder; // encoder to measure the speed of the shooterMotor

  
  public RobotContainer() {
    
    //Joystick+Controller Definitions 
    this.joystickDriver = new Joystick(0); // 'this.' Grabs a variable specifically
    this.joystickShooter = new Joystick(1); // ^^ Creates less confusion in the system
    // The numbers in the parenthesis represents the ports each controller goes to. 
    
    /*
      Start of driver section
    */

    leftDriveMotorLead = new CANSparkMax(Constants.leftDriveMotorLead, MotorType.kBrushless); // Creates new talon motor for leading left drive
    leftDriveMotorLead.setInverted(true); // Inverts Left Drive Motor
    leftDriveMotorLead.set(0); // Sets speed to 0 (anywhere between -1 and 1)

    leftDriveMotorFollow = new CANSparkMax(Constants.leftDriveMotorFollow, MotorType.kBrushless);
    leftDriveMotorFollow.setInverted(true);
    leftDriveMotorFollow.follow(leftDriveMotorLead); 
    
    rightDriveMotorLead = new CANSparkMax(Constants.rightDriveMotorLead, MotorType.kBrushless); // Creates new talon motor for leading right drive
    rightDriveMotorLead.setInverted(true); // Inverts Right Drive Motor
    rightDriveMotorLead.set(0); // Sets speed to 0 (anywhere between -1 and 1)

    rightDriveMotorFollow = new CANSparkMax(Constants.rightDriveMotorFollow, MotorType.kBrushless);
    rightDriveMotorFollow.setInverted(true);
    rightDriveMotorFollow.follow(rightDriveMotorLead);

    leftDriveMotorLeadEncoder = new CANEncoder(leftDriveMotorLead);
    rightDriveMotorLeadEncoder = new CANEncoder(rightDriveMotorLead);
    
    robotDrive = new Drive();
    // It sets a new drive and uses the ints 1 and 2. The order matters.
    // 1 is assigned to leftDriveMotorAddress, whereas 2 is rightDriveMotorAddress
 

    robotDrive.setDefaultCommand(new DriveJoystick(joystickDriver, robotDrive));
    // This helps set the default command. It sets it to DriveJoystick so that way RobotContainer
    // can grab the information and utilize it for the given controller, in this case joystickDriver

    //Motor Definitions

    /*
      Start of intake section
    */
    
   // intakeMotor = new CANSparkMax(Constants.intakeMotorCanAddress, MotorType.kBrushless); //The motor (CANSparkMax) is defined with a type and port (port 5, and motor type = brushless)
    ///intakeMotor =  new TalonSRX(5); //Motor is defined as a specified motor under port five (Talon)
    //intakeMotor.set(0); //Initially sets motor value to 0, will not run without further command

    elevatorMotorLead = new CANSparkMax(Constants.elevatorMotorLeadAddress, MotorType.kBrushless); //Motor is defined under 6th port (CANSparkMax)
    ///elevatorMotorLead = new TalonSRX(6); //Motor is defined (Talon) under port six
    elevatorMotorLead.set(0); //Sets motor value (speed) to 0

    elevatorMotorFollow = new CANSparkMax(Constants.elevatorMotorFollowAddress, MotorType.kBrushless); //Motor is deinfed under 7th port (CANSparkMax)
    ///elevatorMotorFollow = new TalonSRX(7); // Motor is defined under port seven (Talon)
    elevatorMotorFollow.set(0); //Sets motor speed to 0
    elevatorMotorFollow.follow(elevatorMotorLead); // Vertical follow motor will do everthing the vertical lead motor does

    raiseClimbMotor = new CANSparkMax(Constants.raiseClimbMotorAddress, MotorType.kBrushless);
    raiseClimbMotor.set(0);

    telescopicClimbMotor = new CANSparkMax(Constants.telescopicClimbMotorAddress, MotorType.kBrushless);
    telescopicClimbMotor.set(0);
    // The numbers in the parenthesis represents the ports each controller goes to. 
    
    //intakeButton = new JoystickButton(joystickDriver, 5); // Right Upper Bumper, sets intake Button to a controller
    //intakeButton.whileHeld(new IntakeRun());//While the button is being held, the command is being run
  
    elevatorButton = new JoystickButton(joystickDriver, 6); //Left Upper Bumper, elevator button  to a controller
    elevatorButton.whileHeld(new ElevatorRun());//While held, command is being run, references command from commands. Hence imports
    
    intakePneumaticExtendButton = new JoystickButton(joystickDriver, 7);
    intakePneumaticExtendButton.whileHeld(new IntakePneumaticExtend());

    intakePneumaticRetractButton = new JoystickButton(joystickDriver, 8);
    intakePneumaticRetractButton.whileHeld(new IntakePneumaticRetract());
    // Configure the button bindings
   
       
    // autonDriveForw = new JoystickButton(joystickDriver, 3);
    // autonDriveForw.whileHeld(new DriveForwardThreeFeetAuton(1,2, robotDrive));
    /*
      Start of shooter section
    */

    shooterMotor = new CANSparkMax(Constants.shooterMotorCanAddress, MotorType.kBrushed); // instantiates new shooter motor with specific ID

    shooterMotorEncoder = new CANEncoder(shooterMotor); // instantiates a new encoder for the shooterMotor
    
    shooter = new Shooter(); // new Shooter object
    
    pneumatics = new Pneumatics(); //instantiates a new pneuamtics object
    
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
    lowSpeedShooterButton.whenHeld(new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new LowSpeedShooter(this.joystickShooter, shooter),
        new ReleaseGate(this.joystickShooter, pneumatics, Constants.lowShooterSpeed)),
      new CloseGate(this.joystickShooter, pneumatics)));
    /*
      The whenHeld method runs the low speed shooter command when the A button is held.
      The method requires an object of a command, such as new LowSpeedShooter
    */
    
    /*
      see the comment above lowSpeedShooterButton.whenHeld for an explanation
    */
    highSpeedShooterButton = new JoystickButton(this.joystickShooter, 4); // creates the button for the high speed shooter
    highSpeedShooterButton.whenHeld(new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new HighSpeedShooter(this.joystickShooter, shooter),
        new ReleaseGate(this.joystickShooter, pneumatics, Constants.highShooterSpeed)),
      new CloseGate(this.joystickShooter, pneumatics)));
    /*
      The whenHeld method runs the high speed shooter command when the Y button is held.
      The method requires an object of a command, such as new HighSpeedShooter
    */

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