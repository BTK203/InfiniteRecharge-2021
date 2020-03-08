/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.Util;

/**
 * A collection of cameras that help the drivers see
 */
public class CameraHub {
    UsbCamera backCam;

    public CameraHub() {
        new Thread(() -> {
            backCam = CameraServer.getInstance().startAutomaticCapture(0);
            configureCameras();
            CvSink sink = CameraServer.getInstance().getVideo();
            CvSource src = CameraServer.getInstance().putVideo("Pickup", 252, 128);
            Mat img = new Mat();
            while(!Thread.interrupted()) {
                if(sink.grabFrame(img) == 0) {
                    src.putFrame(img);
                } else {
                    DriverStation.reportWarning("COULD NOT READ IMAGE", false);
                }
            }
        }).start();
    }

    public void configResolution() {
        int camResX = (int) Util.getAndSetDouble("Camera Width", 252);
        int camResY = (int) Util.getAndSetDouble("Camera Height", 128);

        backCam.setResolution(camResX, camResY);
    }

    public void configExposure(boolean automatic) {
        if(automatic) {
            backCam.setExposureAuto();
            return;
        }

        int camExposure = (int) Util.getAndSetDouble("Camera Exposure", 50);
        backCam.setExposureManual(camExposure);
    }

    public void configBrightness() {
        int camBrightness = (int) Util.getAndSetDouble("Camera Brightness", 50);
        backCam.setBrightness(camBrightness);
    }

    public void configureCameras() {
        configResolution();
        configExposure(Constants.DRIVE_CAMERA_AUTOMATIC_EXPOSURE);
        configBrightness();
    }
}
