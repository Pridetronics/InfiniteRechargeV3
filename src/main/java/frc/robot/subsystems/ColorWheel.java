/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;

public class ColorWheel extends SubsystemBase {
  // Stores the color the sensor detected
  private Color detectedColor;

  // Stores the color that is required for the game
  private Color requiredColor;

  // Instantiate the I2C sensor
  private I2C.Port i2cPort;

  // Sets up the color sensor
  private ColorSensorV3 colorSensor;

  public ColorWheel(I2C.Port port) {
    // Gives the color sensor the correct port
    i2cPort = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(i2cPort);
  }

  /* Actual color coming in from the wheel sensor */
  public void readSensorColor() {
    /* Call this method continuously in the command to read the values from the sensor and store it in the shorts */
    detectedColor = colorSensor.getColor();
  }

  public double sensorRed() {
    // Returns the red value of the sensor
    return detectedColor.red;
  }

  public double sensorGreen() {
    // Returns the green value of the sensor
    return detectedColor.green;
  }

  public double sensorBlue() {
    // Returns the blue value of the sensor
    return detectedColor.blue;
  }

  public double sensorProximity() {
    // Returns the proximity of the color sensor to the color wheel
    double proximity = colorSensor.getProximity();
    return proximity;
  }

  /* Color required for position stage of color wheel, comes from the specific match */
  public void readRequiredColor() {
    String colordata = DriverStation.getInstance().getGameSpecificMessage();
    // Makes sure the data was actually recieved
    if (colordata.length() > 0) {
      // Does a switch statement based off the first character of the receieved color
      switch (colordata.charAt(0)) {
        case 'B':
          // Sets the required color of the color wheel to blue
          requiredColor = Color.kCyan;
          break;
        case 'G':
          // Sets the required color of the color wheel to green
          requiredColor = Color.kLime;
          break;
        case 'R':
          // Sets the required color of the color wheel to red
          requiredColor = Color.kRed;
          break;
        case 'Y':
          // Sets the required color of the color wheel to yellow
          requiredColor = Color.kYellow;
          break;
        default:
          // Data is corrupt somehow
          break;
      }
    }
  }

  public double requiredRed() {
    // Returns the red value of the required color
    return requiredColor.red;
  }

  public double requiredGreen() {
    // Returns the green value of the required color
    return requiredColor.green;
  }

  public double requiredBlue() {
    // Returns the blue value of the required color
    return requiredColor.blue;
  }

  @Override
  public void periodic() {
    // Nothing
  }
}
