/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /**
   * Creates a new Vision.
   */
  public static Thread visionThread;
  
  public Vision() 
  {
    //Utilizes vision processing for better alignment when shooting and video with Limelight
    
    visionThread = new Thread(() ->
    {
      UsbCamera lifeCam = CameraServer.getInstance().startAutomaticCapture();
      lifeCam.setResolution(Constants.lifeCamWidth, Constants.lifeCamHeight);

      CvSink cvsink = CameraServer.getInstance().getVideo();

      CvSource outputStream = 
        CameraServer.getInstance().putVideo("Front Camera", Constants.lifeCamWidth, Constants.lifeCamHeight);

      Mat mat = new Mat();

      while(!Thread.interrupted())
      {
        if(cvsink.grabFrame(mat) == 0)
        {
          outputStream.notifyError(cvsink.getError());
          continue;
        }
        /*
          Any image processing that may want to be done will be done here
        */
        outputStream.putFrame(mat);
      }
    });
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
