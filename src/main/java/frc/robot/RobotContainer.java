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
import frc.robot.commands.LowSpeedShooter;
import frc.robot.commands.ReleaseGate;
import frc.robot.commands.HighSpeedShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
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
    
  public Joystick joystickDriver; //The name of the first controller, main driver
  public Joystick joystickShooter; //The name of the second controller, secondary driver
  
  public JoystickButton intakeButton; //Button to run the intake 
  public JoystickButton elevatorButton; //Button to run the intake Vertical
  
  public final Drive robotDrive;

  public JoystickButton lowSpeedShooterButton; // Button A
  public JoystickButton highSpeedShooterButton; // Button Y

  public Shooter shooter; // shooter object to be used for shooter commands

  public static CANSparkMax shooterMotor;

  public Pneumatics pneumatics;
  
  public static DoubleSolenoid shooterBallRelease;
  public static DoubleSolenoid intakeDeploy;
  public static DoubleSolenoid controlPanelSpinnerDeploy;

  public static CANEncoder shooterMotorEncoder;


  public RobotContainer() {
    /*
    Start of Drive section
    */

    this.joystickDriver = new Joystick(0); // 'this.' Grabs a variable specifically
    this.joystickShooter = new Joystick(1); // ^^ Creates less confusion in the system
    // The numbers in the parenthesis represents the ports each controller goes to. 
    
    robotDrive = new Drive(1, 2); 
    // It sets a new drive and uses the ints 1 and 2. The order matters.
    // 1 is assigned to leftDriveMotorAddress, whereas 2 is rightDriveMotorAddress

    robotDrive.setDefaultCommand(new DriveJoystick(joystickDriver, robotDrive));
    // This helps set the default command. It sets it to DriveJoystick so that way RobotContainer
    // can grab the information and utilize it for the given controller, in this case joystickDriver
    
    /*
    Start of Shooter section
    */
    
    shooterMotor = new CANSparkMax(2, MotorType.kBrushed); // instantiates new shooter motor with specific ID

    shooterMotorEncoder = new CANEncoder(shooterMotor);
    
    shooter = new Shooter(); // new Shooter object
    
    pneumatics = new Pneumatics();
    
    lowSpeedShooterButton = new JoystickButton(this.joystickShooter, 1); // creates the button for the low speed shooter
    //lowSpeedShooterButton.whenHeld(new LowSpeedShooter(this.joystickShooter, shooter));
    lowSpeedShooterButton.whenHeld(new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new LowSpeedShooter(this.joystickShooter, shooter),
        new ReleaseGate(this.joystickShooter, pneumatics)),
      new CloseGate(this.joystickShooter, pneumatics)));
    /*The whenHeld method runs the low speed shooter command when the A button is held.
    The method requires an object of a command, such as new LowSpeedShooter
    */
    
    highSpeedShooterButton = new JoystickButton(this.joystickShooter, 4); // creates the button for the high speed shooter
    highSpeedShooterButton.whenHeld(new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new HighSpeedShooter(this.joystickShooter, shooter),
        new ReleaseGate(this.joystickShooter, pneumatics)),
      new CloseGate(this.joystickShooter, pneumatics)));
    /*The whenHeld method runs the high speed shooter command when the Y button is held.
    The method requires an object of a command, such as new HighSpeedShooter
    */

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