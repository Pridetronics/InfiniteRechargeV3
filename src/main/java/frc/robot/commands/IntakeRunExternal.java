/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Talon;
public class IntakeRunExternal extends CommandBase {
  /**
   * Creates a new IntakeRunExternal.
   */
  public Intake m_Intake;
  private CANSparkMax intakeMotorExternal = Intake.intakeMotorExternal;
  //private Talon intakeMotorExternal = Intake.intakeMotorExternal;

  public IntakeRunExternal() {

    addRequirements(m_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }
  public void intakeRunExternal()
  {

    SmartDashboard.putBoolean("Intake External", true);
    intakeMotorExternal.set(55);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
