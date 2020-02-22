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
    public static final int LEFT_DRIVE_MOTOR_LEAD = 1;
    public static final int RIGHT_DRIVE_MOTOR_LEAD = 2;
    public static final int INTAKE_MOTOR_CAN_ADDRESS = 5;
    public static final int ELEVATOR_MOTOR_CAN_ADDRESS = 7;
    public static final int RAISE_CLIMB_MOTOR_ADDRESS = 8;
    public static final int TELESCOPIC_CLIMB_MOTOR_ADDRESS = 9;
    public static final int SHOOTER_MOTOR_CAN_ADDRESS = 2; // creates can address for shooter motor

    public static DifferentialDrive robotDrive; // Creates new differential drive

    public static final int SHOOTER_GATE_FORWARD_CHANNEL = 5;
    public static final int SHOOTER_GATE_RELEASE_CHANNEL = 4;
    public static final int INTAKE_SOLENOID_FORWARD_CHANNEL = 0;
    public static final int INTAKE_SOLENOID_REVERSE_CHANNEL = 1;

    public static final int INTAKE_LIMIT_SWITCH_CHANNEL = 1;
    public static final int SHOOTER_LIMIT_SWITCH_CHANNEL = 1;
    public static final int UPPER_CLIMB_LIMIT_CHANNEL = 0;
    public static final int LOWER_CLIMB_LIMIT_CHANNEL = 1;

    public static final double INTAKE_MOTOR_SPEED = 0.55;
    public static final double TELESCOPIC_ROD_MOTOR_SPEED = 0.2;
    public static final double INVERSE_TELESCOPIC_MOTOR_SPEED = -0.2;
    public static final double WINCH_MOTOR_SPEED = 0.2;

    public static final double WINCH_TIMEOUT = 5.0;

    public static final double SHOOTER_DESIRED_RPM = 3300.0/5767.0;

    // PID Setup
    public static final double lowShooterSpeed = 3300.; // creates the speed for the LowSpeedShooter
    public static final double Kp = 0.0002;
    public static final double Ki = 0.000001;
    public static final double Kd = 0.0004;
    // public static final double kIz = 0; 
    // public static final double kFF = 0.000156; 
    // public static final double kMaxOutput = 1; 
    // public static final double kMinOutput = -1;

    public static void init() {


    
        // LiveWindow.addAcutator("Drive", "robotDrive", myRobot);
        robotDrive.setSafetyEnabled(false);
        robotDrive.setExpiration(0.1);
        robotDrive.setMaxOutput(1.0);
    }
}
