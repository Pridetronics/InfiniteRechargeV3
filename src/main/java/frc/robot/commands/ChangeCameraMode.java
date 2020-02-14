/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ChangeCameraMode extends CommandBase {
  /**
   * Creates a new ChangeCameraMode.
   */
  public static VideoSink server;
  public static UsbCamera m_USBCamera1;
  public static UsbCamera m_USBCamera2;
  int cameraMode;

  public ChangeCameraMode() {
    // Use addRequirements() here to declare subsystem dependencies.
    
    m_USBCamera1 = RobotContainer.USBCamera1;
    m_USBCamera2 = RobotContainer.USBCamera2;
    server = CameraServer.getInstance().getServer();
    cameraMode = 1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(cameraMode == 1)
    {
      System.out.println("Switching to Camera 2");
      server.setSource(m_USBCamera2);
      cameraMode = 2;
    }
    else
    {
      System.out.println("Switching to Camera 1");
      server.setSource(m_USBCamera2);
      cameraMode = 1;
    }
    
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
