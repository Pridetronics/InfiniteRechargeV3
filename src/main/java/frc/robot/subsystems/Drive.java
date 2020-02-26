/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
XX0KNNNNNNNNNNNXXNNNNNNNNNNNNNNNNNNNNNNNNNWWNNNXK0OkkkkkkkkkkkOKNNWWWWWNKXWWWWWWNNNXNWWWWWWWWWWWWWWWWWWWWWWMWW
XX0KNNNNNNNNNNNXXNNNNNNNNNNNNNNNNNNNNNNNNNKOdlcc::;,,,,,,,;;;::cloxOKNWNKKWWWWWWNNNXNWWWWWWWWWWWWWWWWWWWWWWMWW
XX0KNNNNNNNNNNNXXNNNNNNNNNNNNNNNNNNNNNKOo:,'',;cccccc:::cclc:;;:;'',:lxkOKWWWWWWNNNXNWWWWWWWWWWWWWWWWWWWWWWWWW
XX0KNNNNNNNNNNNXXNNNNNNNNNNNNNNNNNWXOo;..'',,,,;;;,;,,,,'',;,;c;'...,;;:cdOXNWWWNNNXNWWWWWWWWWWWWWWWWWWWWWWWWW
XX0KNXNNNNNNNNNXXNNNNNNNNNNNNNWNNKx:....,,'..................',,'...',;:cccokKWWNNNXNWWWWWWWWWWWWWWWWWWWWWWWWW
XXKKNNNNNNNNNNNXXNNNNNNNNNNNNNWXx;....',,'.................',,;;'....;:ccclllokKNNNXNWWWWWWWWWWWWWWWWNWWWWWWWW
XXKKNNNNNNNNNNNXXNNNNNNNNNNNNNOc.. ..,;;,'''''............',;;;;,''.',:loollll;ckKNXNWWWWWWWWWWWWWWWWNWWWWWWWW
XXKKNNNNNNNNNNNXXNNNNNNNNNNNNx;.  .';;;,''''''''''''''..',;:ccclccc::clc:clllc:;;xXXNWWWWWWWWWWWWWWWWNWWWWWWWW
XXKKNNNNNNNNNNNXXNNNNNNNNNNXd,....',,,,,;;;:::::::::ccllloddxxxxkxxkkkkollccccccclOXNWWWWWWWWWWWWWWWWNNWWWWWWW
XNKKXNNNNNNNNNNXXNNNNNNNNNXd,. ..'',;;:clloddddddddxxkkkkkkOOOOOOOO0000kxxolllccl;,xNWWWWWWWWWWWWWWWWNNWWWWWWW
NXKKXNNNNNNNNNNXXNNNNNNNNNk:'. .,,;;:lodxxkkkkOOOOOOOOOOOOO0OOOOO00000KK00Odlcccl:.'OWWWWWWWWWWWWWWWWNNWWWWWWW
NNK0XNNNNNNNNNNNXNNNNNNNKkxdOx,';;:cdxkOOOOOOO000000000000000OOO00OO00KKKXK0xolll:,,oKWWWWWWWWWWWWWWWNNWWWWWWW
NNK0XNNNNNNNNNNNXNNNNNN0:.'oXO;';;:oxkOOOOOOO0000000000O0000OOO00000KKKKXXXK0xoccc:okxkOXWWWWWWWWWWWWNNWWWWWWW
NNK0XNNNNNNNNNNNXNNNNNXo'.;OXo'.,;cdkOOOOOOOO0000O00OO00000OOO0000KKKXXXXXXK0kolc:;lxolldKWWWWWWWWWWWNNWWWWWWW
XNK0XNNNNNNNNNNNXXNNNNKc'.;l,...':ldkOOOOOOOOOOOOOOOO000000000OO0000KKXNNXXK0koc:;,'lo:loONWWWWWWWWWNNNWWWWWWW
XNK0XNNNNNNNNNNXXXNNXkl;'.     .';lxOOkkxxddooooodxxkO0000OOOkkxxddxxxk0KKXXKOdc;,.  ..;cxKNWWWWWWWWNNNWWWWWWW
XNK0XNNNNNNNNNNXXXNNx,.''.     ..;okkxdddddooloooddxxkO000OOkddddoooodddxxO000xc,'.    .:odkKWWNWWWWNNNWWWWWWW
XNK0XNXXXNNNNNNXXXNXo...'.      .;dkxxkkkkxdoloodxkkkOO00OOkkxxxxxxdddxxxkkk00Ol,.     .:lllxXWNWWWWNNNNWWWWWW
XXK0XNXXXXXXXNNNXXXKo'....      .:xkkOOOxooc,,,cdxkxxkkOOOkxddddxdc;;;codxkOO00x;.     .,;c:oXWNNWWWNNNNNNNNWW
XXK0KXXXXXXXXNNNNNNNd'....      .:xOOOOkxdxl;,;oxxxxxxkOO0KOdodoodc,''coodkO00Kk:.     .'','lXWNNWWWNNNXNNXXNW
XXK0KXXXXXXXXNNNNNNNx'..... .   .ckO0OOOOkkxdddxxddxxkOO0KXKkooooddolodxkO00KKXOc.     .''..oNWNNNWWWWWWWWWWWW
XXK0KXXXXXXXXXNNNNXNk,........  .okO00OOOOOOOOkxxxxxkkkOOKXX0kddddxkkO00KXXXXXX0l.    ..'...dNNNNNNWWWWWWWWWWW
XXK0KXXXXXXXXXXXXNNN0;........  .okO00OOOOOOOOkkOOkkkkkO0KXXKOkxkkkO00KKXXNNXXK0l.  ....'..'ONNNNNNNNNNNNNNNNN
XXKOKXXXXXXXXXXXXXXNKc...''..'. .oOO000000OOOOOOOOOkkkOOO0KXX0OOOO0KK00KKKKKKKK0:  ........:KWNNNNNNNNNNNNNNNN
dO0OKXXXXXXXXXXXXXXNXd'..,:,',' .ckO000000000OOOOOOkkO0000KKK0OOOO0000000000000k,  ........oNWNNNNNNNNNNNNNNNN
lOKOKXXXXXXXXXXXXXXXNO;..':;.,,. ,xO0000000000000OkkOkO0000OkOOkkO0000000000000o. ....,.. 'ONNNNNNNNNNNNNNNNNN
0XKOKXXXXXXXXXXXXXXXNXo..','..'. .oOO000000000000OOOkxxOOkkxdkO0OOO000000OOOOOk;  ...',...lXNNNNNNNNNNNNNNNNNN
XXKOKXXXXXXXXXXXXXXXXXO,........  ckOO000000000OOOkkkkO0OOOkk0KKK0OOO0000OOkkOo.    .... .d0O0NNNNNNNNNNNNNNNN
KXKOKXXXXXXXXXXKKXXXXXKo......    ,xOOO00000000OOOkOOOOOOOO0000KKK000OOOOOkxkx:      ..  .';,;dXNNNNNNNNNNNNNN
KXKOKXXXXXXXXXXXXXNNNNN0:.....    .oOOOO00000OOOOOOOOO00OkO0OOOO000K00OOOxoodo.      .. .. ..,dXNNNNXXXXXXXXXX
KXKOKXXXXXXXXXXXKXXXXXXXx. ...     cOOOOOO0OOOOOOOOOOO0OOkOkkkkkkkOOO0Od:......      . 'o; .o0KNNNNNNNWWWWWWWW
KKK0KXXXXXXXXXXXKXXXXXXXXkc'...,::coOOOOOOOOOOOOkxdddoooooooooolldxkkOx,    .....   ..'xK:.lXNNNNNNNNXNNNNNNNN
KKK0KXXXXXXXXXXXKXXXXXXXXXX0o':0XXXX0OOOOOOOOOOkkxkxxkkkxxxxkOOkkxxkxkd'      ..;;..;xKN0:cKNNNNNNNNNXXNNNNNNN
KKK0KXKXXXXXXXXXKKXXXXXXXXXXXkoOXKXX0kkkkkOOOOOOOO0000OOOOkkOOOOOOOkkkkd,.     .oo,d0XNKl:ONNNNXNNNNNXXXNNNNNN
KKK0KXKXXXXXXXXXKKXXXXXXXKXXXKdxKKXX0kkkkkkkOOOOOOO00OOOOOOOOOOOOOOkkkkkxl::ccc;:cxKKOkxxdkXNNXXXXNNNXXXNNNNNN
KKK0KXXKKXXXXXXXKKXXXXXXXXXXXX0dkKXX0kkkkkkkkkOOOOOOOOOOOOOOOOOOOOOOOOOkxod0NNN0c;ododkKXkkXNNXNXXNNNXXXXNNNNN
KKK0KXKKKKKKXXXXKKXXXXXXKKKKKKKxlxxoxkOOOOkkkkkOOOOOOOOOOOOO00000000OOkxdodOXXXKdx0OOKNXXOxKNXNNXXNNNXXXXNNNNN
KKK0KKKKKKKKKK00OOOkkxxxdddoolc;.. .lkOOOOkkxkkOOOOOO00O00OOOO0000OOOkxxxddlllol:okkkO0KXOd0NXNNXXNNNXXXXNNNNN
KKK0O0OOkxxdddoooooooooooolc;.    .,dkkOOOOOkxxxkOOOOOOOOOOOOOOOkkkxxxxxddd, ...':ccccllll:lxkO0KKXNNXXXXXXNNN
kxddooooooooododddollllc;'..     .;oxkkOOOOOkkxxxxkkkOOOOOOOOkkxxdddxxxxxxd'    ..,:loolll;;ccccllodkO0KXNXXNN
oooooddddddddol:,..  ..         'olcdkkkOOOOkkkkkkkkkkkkkkkkxxxdooodxxxxxxxc.      ..,;:cl:;clllllc:cccldkKXNN
ddddddooooool;.                 :xdclxkkOOOOkkkkkkOOOOOO00OOOkxdoodxxxxxxxkk;...     ....,,,;cccccllollcc:coOX
oooooooolc:;.                   ,xkocdkkOOOOOkkkkkkOOOO000OOkxdooodxxxxxxkO0l. .      ......';;::::ccccllll:;o
ollllll:'.                      .lkxllxkkOOOOOkkkkkkkkOOOOOkxddodddxxxxxxkOO:  .       .......'',,;;;::::ccl;'
llcc:;'.                         'dkdlokkkOOOOOkkkkkkkkkkkkxxddddxxxxxxxxkOl.          ............',,;;;;:::'
;'..                              ,dkdldkkkOOOOOOkkOOOkkkkkxxxxxxxxxxxxxkxc.            ... ..........'',,,;;.
                                   ;xkolxkkkOOOOOOOOOOOOOkkkxxxkkkxxxkkkd:.              .. ...     ..........
            ..                      :kxldOkkOOOOOOOOOkkkkkkkxxkkkkkkkkkd:.   ..          ..  ..      .........
*FRC when they create an industrial-grade library to help you code your robot, just for you to not use it
*and steal their code instead
*/

package frc.robot.subsystems;

import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
    //m_odometry.update(Rotation2d.fromDegrees(getMeasurement()), leftDriveMotorEncoder.getPosition(),rightDriveMotorEncoder.getPosition());

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
    //robotDrive.tankDrive(leftValue, rightValue, squareInputs);

    // // Checks that the value is between -1 and 1
     leftValue = MathUtil.clamp(leftValue, -1.0, 1.0);
     rightValue = MathUtil.clamp(rightValue, -1.0, 1.0);
    
    // // Creates a deadzone on the controller to reduce drive jitter
     leftValue = applyDeadband(leftValue, Constants.DEADBAND);
     rightValue = applyDeadband(rightValue, Constants.DEADBAND);

    // // Squares the input to make it a exponential response curve instead of linear
    // // to increase fine control while permitting full power
     if (squareInputs) 
     {
       // Squares the values and copies the sign from the initial value
      // This makes sure that if the values were negative that they stay negative after the square
      leftValue = Math.copySign(leftValue * leftValue, leftValue);
      rightValue = Math.copySign(rightValue * rightValue, rightValue);
    }
    
    // Converts the percentage value to RPM for the PID Loop
    leftValue *= Constants.MAX_SHOOTER_RPM;
    rightValue *= Constants.MAX_SHOOTER_RPM;

    // Sets the reference point on the PID loop to the specified RPM
    m_leftDrive_pid.setReference(leftValue, ControlType.kVelocity);
    m_rightDrive_pid.setReference(rightValue, ControlType.kVelocity);
  }

  public void resetAngle() {
    /* Resets the yaw to 0 to wherever the robot is pointed */
    /* Only reccomended to press once at the start of the match once the robot is in place to calibrate */
    /* Please be very careful with this, it's extremely important that the forward direction is 0 for the autonomous to work correctly */
    navX.zeroYaw();
  }

  public double getRotationRate(){
    return rotateToAngleRate;
  }

  public void zeroRotationRate(){
    rotateToAngleRate = 0;
  }

  public boolean atSetPoint() {
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
