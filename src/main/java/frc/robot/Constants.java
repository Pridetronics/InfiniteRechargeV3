/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
/* import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Victor; */
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import com.revrobotics.CANPIDController;

//import com.revrobotics.ControlType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static CANSparkMax leftDriveMotorLead; // Creates new talon motor for leading left drive
    public static CANSparkMax rightDriveMotorLead; // Creates new talon motor for leading right drive

    public static DifferentialDrive robotDrive; // Creates new differential drive
  
    public static void init() {


    
        // LiveWindow.addAcutator("Drive", "robotDrive", myRobot);
        robotDrive.setSafetyEnabled(false);
        robotDrive.setExpiration(0.1);
        robotDrive.setMaxOutput(1.0);
    }
}
