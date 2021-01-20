// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class PathRecorder {
    String file;
    BufferedWriter buffer;
    FileWriter writer;
    Point2D lastPoint;
    long lastFlushTime;
    
    public PathRecorder(String file) {
        this.file = file;
        lastPoint = new Point2D(0, 0, 0);
        lastFlushTime = System.currentTimeMillis();
        openFile();
    }

    public void openFile(){
        try {
            writer = new FileWriter(file, false);
            buffer = new BufferedWriter(writer);
        } catch (IOException ex) {
            DriverStation.reportError("IO EXCEPTION", true);
        }
    }

    public void closeFile() {
        try {
            writer.close();
            buffer.close();
        } catch(Exception ex) {
            DriverStation.reportError("IO EXCEPTION", true);
        }
    }

    public void recordPoint(Point2D point) {
        try {
            if(point.getDistanceFrom(lastPoint) >= Constants.PATH_RECORDER_DISTANCE_INTERVAL) {
                buffer.append(point.toString() + "\n");
            }

            //decide if we should flush
            long currentTime = System.currentTimeMillis();
            if(currentTime - lastFlushTime > 1000) {
                flushFile();
                lastFlushTime = currentTime;
            }
        } catch(IOException ex) {
            DriverStation.reportError("IO EXCEPTION OCCURRED", true);
        }
    }

    public void flushFile() throws IOException {
        buffer.flush();
    }

    private String pointArrayToString(ArrayList<Point2D> points) {
        String retval = "";
        for(int i=0; i<points.size(); i++) {
            retval += points.get(i).toString();
            if(i < points.size() - 1) {
                retval += ",\n";
            }
        }

        return retval;
    }
}
