/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wipilibj.Talon;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  public static CANSparkMax intakeMotorExternal;
  public static CANSparkMax intakeMotorVertical;
  //private Talon intakeMotorExternal;
  //private Talon intakeMotorVertical;


   public Intake() {
    //Collect power cell balls
    intakeMotorExternal = new CANSparkMax(5, MotorType.kBrushless);
    //intakeMotorExternal =  new Talon(5);
    intakeMotorExternal.setInverted(true); 
    intakeMotorExternal.set(0);

    intakeMotorVertical = new CANSparkMax(6, MotorType.kBrushless);
    //intakeMotorVertical = new Talon(6);
    intakeMotorVertical.setInverted(true);
    intakeMotorVertical.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
