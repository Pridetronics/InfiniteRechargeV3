/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //motor addresses
    public static final int LEFT_DRIVE_MOTOR_LEAD = 1;
    public static final int RIGHT_DRIVE_MOTOR_LEAD = 2;
    public static final int LEFT_DRIVE_MOTOR_FOLLOW = 3;
    public static final int RIGHT_DRIVE_MOTOR_FOLLOW = 4;
    public static final int INTAKE_MOTOR_CAN_ADDRESS = 5;
    public static final int ELEVATOR_MOTOR_CAN_ADDRESS = 7;
    public static final int RAISE_CLIMB_MOTOR_ADDRESS = 8;
    public static final int TELESCOPIC_CLIMB_MOTOR_ADDRESS = 9;
    public static final int SHOOTER_MOTOR_CAN_ADDRESS = 10;

    //Button numbers
    public static final int SHOOTER_BUTTON_NUMBER = 1;
    public static final int INTAKE_BUTTON_NUMBER = 5;
    public static final int TELESCOPIC_ROD_BUTTON_NUMBER = 7;
    public static final int LIFT_ROBOT_BUTTON_NUMBER = 5;

    //differential drive object, might delete because we don't use it anymore
    public static DifferentialDrive robotDrive;

    //forward and reverse channels for double solenoids
    public static final int SHOOTER_GATE_FORWARD_CHANNEL = 2;
    public static final int SHOOTER_GATE_RELEASE_CHANNEL = 3;
    public static final int INTAKE_SOLENOID_FORWARD_CHANNEL = 0;
    public static final int INTAKE_SOLENOID_REVERSE_CHANNEL = 1;

    //limit switch channels for intake, shooter, and climb
    public static final int INTAKE_LIMIT_SWITCH_CHANNEL = 1;
    public static final int SHOOTER_LIMIT_SWITCH_CHANNEL = 1;
    public static final int UPPER_CLIMB_LIMIT_CHANNEL = 0;
    public static final int LOWER_CLIMB_LIMIT_CHANNEL = 1;

    //Left Drive PID Setup
    public static final double LEFT_DRIVE_kP = 0.00002;
    public static final double LEFT_DRIVE_kI = 0.000001;
    public static final double LEFT_DRIVE_kD = 0.0;

    //Right Drive PID Setup
    public static final double RIGHT_DRIVE_kP = 0.00002;
    public static final double RIGHT_DRIVE_kI = 0.000001;
    public static final double RIGHT_DRIVE_kD = 0.0;

    //Motor speeds
    public static final double INTAKE_MOTOR_SPEED = 0.55;
    public static final double TELESCOPIC_ROD_MOTOR_SPEED = 0.2;
    public static final double INVERSE_TELESCOPIC_MOTOR_SPEED = -0.2;
    public static final double WINCH_MOTOR_SPEED = 0.2;

    //Timeout for winch
    public static final double WINCH_TIMEOUT = 5.0;

    // Auto Setup
    public static final double AUTO_TRAVEL_SPEED = -0.5; // Don't put above 0.75, needs some room for turning corrections
    
    // Gear Ratios
    public static final double MAIN_MOTOR_RATIO = 7.31; // 1 : 7.31 gear ratio
    
    // Distance Calculations Setup
    public static final double WHEEL_CIRCUMFERENCE = 0.1524 * Math.PI; // 0.1524 meters * PI = Wheel Circumfrence in meters
    
    // Drive Turning PID Setup
    public static final double TURN_kP = 0.0001;
    public static final double TURN_kI = 0.000001;
    public static final double TURN_kD = 0;
    public static final double TURN_TOLERANCE = 5.0;
    public static final double TURN_PS_TOLERANCE = 10.0;
    
    // Shooter PID Setup
    public static final double SHOOTER_LOW_SPEED = 3300.0; // creates the speed for the LowSpeedShooter
    public static final double SHOOTER_kP = 0.0002;
    public static final double SHOOTER_kI = 0.000001;
    public static final double SHOOTER_kD = 0.0004;
    public static final double SHOOTER_DESIRED_RPM = 3300.0 / 5767.0;
    public static final double MAX_NEO_RPM = 5676.0;

    //Drive PID Setup
    public static final double DRIVE_kP = 0.0001;
    public static final double DRIVE_kI = 0.000001;
    public static final double DRIVE_kD = 0;
    
    //Deadzone for TankDrive method
    public static final double DEADBAND = 0.02;

    public static void init() {
        // LiveWindow.addAcutator("Drive", "robotDrive", myRobot);
        robotDrive.setSafetyEnabled(false);
        robotDrive.setExpiration(0.1);
        robotDrive.setMaxOutput(1.0);
    }
}
