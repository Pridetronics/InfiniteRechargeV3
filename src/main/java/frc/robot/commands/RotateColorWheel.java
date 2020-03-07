/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorWheel;

public class RotateColorWheel extends CommandBase 
{
  /**
   * Creates a new RotateColorWheel.
   */
  //Creates new ColorWheel object
  ColorWheel m_colorWheel;
  
  //Creates color object
  private Color sensedColor;

  //Counter to keep track of how many times a certain color has appeared
  private int counter;

  public RotateColorWheel(ColorWheel colorWheel) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    //References ColorWheel object from robot container
    m_colorWheel = colorWheel;

    addRequirements(m_colorWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    //Reads the color that is currently under the sensor
    m_colorWheel.readSensorColor();

    //Constructs the color under the sensor
    sensedColor = ColorMatch.makeColor(m_colorWheel.sensorRed(), m_colorWheel.sensorGreen(), m_colorWheel.sensorBlue());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //Runs the color wheel motor at a certain speed
    m_colorWheel.runColorWheelMotor(Constants.COLOR_WHEEL_MOTOR_SPEED);

    //Updates what the detectedColor is
    m_colorWheel.readSensorColor();

    //Adds 1 to the counter if the original sensed color is equal to the current detected color
    if(m_colorWheel.compareSensedColor(sensedColor))
    {
      counter++;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    //Stops the color wheel motor
    m_colorWheel.runColorWheelMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    // Once the counter does 8 rotations then end command
    return (counter >= 8);
  }
}
