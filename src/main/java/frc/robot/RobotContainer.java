/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveJoystick;
import frc.robot.subsystems.Drive;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.ElevatorRun;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
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
  public static CANSparkMax intakeMotor;
  public static CANSparkMax elevatorMotorLead;
  public static CANSparkMax elevatorMotorFollow;
  public static CANSparkMax raiseClimbMotor;
  public static CANSparkMax telescopicClimbMotor;

  public RobotContainer() 
  {

    //Motor Definitions

    intakeMotor = new CANSparkMax(5, MotorType.kBrushless); //The motor (CANSparkMax) is defined with a type and port (port 5, and motor type = brushless)
    ///intakeMotor =  new Talon(5); //Motor is defined as a specified motor under port five (Talon)
    intakeMotor.set(0); //Initially sets motor value to 0, will not run without further command

    elevatorMotorLead = new CANSparkMax(6, MotorType.kBrushless); //Motor is defined under 6th port (CANSparkMax)
    ///elevatorMotorLead = new Talon(6); //Motor is defined (Talon) under port six
    elevatorMotorLead.set(0); //Sets motor value (speed) to 0

    elevatorMotorFollow = new CANSparkMax(7, MotorType.kBrushless); //Motor is deinfed under 7th port (CANSparkMax)
    ///elevatorMotorFollow = new Talon(7); // Motor is defined under port seven (Talon)
    elevatorMotorFollow.set(0); //Sets motor speed to 0
    elevatorMotorFollow.follow(elevatorMotorLead); // Vertical follow motor will do everthing the vertical lead motor does

    raiseClimbMotor = new CANSparkMax(7, MotorType.kBrushless);
    raiseClimbMotor.set(0);

    telescopicClimbMotor = new CANSparkMax(8, MotorType.kBrushless);
    telescopicClimbMotor.set(0);
  
    //Joystick+Controller Definitions 
    this.joystickDriver = new Joystick(0); // 'this.' Grabs a variable specifically
    this.joystickShooter = new Joystick(1); // ^^ Creates less confusion in the system
    // The numbers in the parenthesis represents the ports each controller goes to. 
    
    robotDrive = new Drive(1, 2); 
    // It sets a new drive and uses the ints 1 and 2. The order matters.
    // 1 is assigned to leftDriveMotorAddress, whereas 2 is rightDriveMotorAddress

    robotDrive.setDefaultCommand(new DriveJoystick(joystickDriver, robotDrive));
    // This helps set the default command. It sets it to DriveJoystick so that way RobotContainer
    // can grab the information and utilize it for the given controller, in this case joystickDriver
    intakeButton = new JoystickButton(joystickDriver, 6); // Right Upper Bumper, sets intake Button to a controller
    intakeButton.whileHeld(new IntakeRun());//While the button is being held, the command is being run
  
    elevatorButton = new JoystickButton(joystickDriver, 5); //Left Upper Bumper, elevator button  to a controller
    elevatorButton.whileHeld(new ElevatorRun());//While held, command is being run, references command from commands. Hence imports
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