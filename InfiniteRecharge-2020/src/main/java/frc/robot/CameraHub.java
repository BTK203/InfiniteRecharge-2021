/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.util.Util;

/**
 * A collection of cameras that help the drivers see
 */
public class CameraHub {
    UsbCamera
        frontCam,
        backCam,
        climberCam;

    public CameraHub() {
        frontCam   = new UsbCamera("Front", 0);
        backCam    = new UsbCamera("Back",  1);
        climberCam = new UsbCamera("Climber", 2);

        configureCameras();

        CameraServer.getInstance().addCamera(frontCam);
        CameraServer.getInstance().addCamera(backCam);
        CameraServer.getInstance().addCamera(climberCam);
    }

    public void configResolution() {
        int camResX = (int) Util.getAndSetDouble("Camera Width", 80);
        int camResY = (int) Util.getAndSetDouble("Camera Height", 55);

        frontCam.setResolution(camResX, camResY);
        backCam.setResolution(camResX, camResY);
        climberCam.setResolution(camResX, camResY);
    }

    public void configExposure(boolean automatic) {
        if(automatic) {
            frontCam.setExposureAuto();
            backCam.setExposureAuto();
            climberCam.setExposureAuto();
            return;
        }

        int camExposure = (int) Util.getAndSetDouble("Camera Exposure", 50);
        frontCam.setExposureManual(camExposure);
        backCam.setExposureManual(camExposure);
        climberCam.setExposureManual(camExposure);
    }

    public void configBrightness() {
        int camBrightness = (int) Util.getAndSetDouble("Camera Brightness", 50);
        frontCam.setBrightness(camBrightness);
        backCam.setBrightness(camBrightness);
        climberCam.setBrightness(camBrightness);
    }

    public void configureCameras() {
        configResolution();
        configExposure(Constants.DRIVE_CAMERA_AUTOMATIC_EXPOSURE);
        configBrightness();
    }
}
