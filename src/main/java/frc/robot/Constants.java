/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
/* import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
 import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Victor; 
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Victor; 

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
    public static final int leftDriveMotorLead = 1;
    public static final int rightDriveMotorLead = 2;
    public static final int intakeMotorCanAddress = 5;
    public static final int elevatorMotorLeadAddress = 6;
    public static final int elevatorMotorFollowAddress = 7;
    public static final int raiseClimbMotorAddress = 8;
    public static final int telescopicClimbMotorAddress = 9;
    //changed for testing
    public static final int shooterMotorCanAddress = 2; // creates can address for shooter motor

    public static DifferentialDrive robotDrive; // Creates new differential drive

    public static final double lowShooterSpeed = 0.; // creates the speed for the LowSpeedShooter

    //These constants will be changed when I found out the actual channels
    public static final int shooterGateForwardChannel = 5;
    public static final int shooterGateReverseChannel = 4;

    public static final int intakeLimitSwitchChannel = 1;
    public static final int shooterLimitSwitchChannel = 1;
    public static final int upperClimbLimitChannel = 6;
    public static final int lowerClimbLimitChannel = 5;

    public static final double TELESCOPIC_ROD_MOTOR_SPEED = 0.6;
    public static final double INVERSE_TELESCOPIC_MOTOR_SPEED = -0.6;
    public static final double WINCH_MOTOR_SPEED = 0.5;

    public final double shooterSpeed = 3300.0/5676.0;

    public static void init() {


    
        // LiveWindow.addAcutator("Drive", "robotDrive", myRobot);
        robotDrive.setSafetyEnabled(false);
        robotDrive.setExpiration(0.1);
        robotDrive.setMaxOutput(1.0);
    }
}
