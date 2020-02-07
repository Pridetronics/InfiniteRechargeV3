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

  public Pneumatics pneumatics; // creates a pneumatic object
  
  public static DoubleSolenoid shooterBallRelease; // represents the solenoids for the different intake systems
  public static DoubleSolenoid intakeDeploy;
  public static DoubleSolenoid controlPanelSpinnerDeploy;

  public static CANEncoder shooterMotorEncoder; // encoder to measure the speed of the shooterMotor


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