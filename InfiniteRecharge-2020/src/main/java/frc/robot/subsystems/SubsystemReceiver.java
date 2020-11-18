/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The thing that listens to the Pi.
 */
public class SubsystemReceiver extends SubsystemBase {
  private String latestSegment;
  private double[] latestData;

  private Boolean inRange;

  private DatagramSocket serverSocket;
  private byte[]         receiveData;

  private long latestTime;


  /**
   * Creates a new SubsystemReceiver.
   */
  public SubsystemReceiver() {
    latestSegment = "-1,-1,-1,-1,-1,180,180";
    latestData = new double[] {-1, -1, -1, -1, -1, 180, 180};
    latestTime    = System.currentTimeMillis();

    SmartDashboard.putString("RPi Data", latestSegment);
    SmartDashboard.putBoolean("Spotted", false);
    SmartDashboard.putBoolean("Updated", false);
    inRange = false;

    try {
      serverSocket = new DatagramSocket(3695);
      receiveData  = new byte[1024];
    } catch (SocketException e) { //thrown when a socket cannot be created
      DriverStation.reportError("SOCKET EXCEPTION", true);
    }

    // EXPECTED FORMAT OF INPUT STRING:
    // :X,Y,H,D,A;
      // X = X-coordinate
      // Y = Y-coordinate
      // D = Distance from target
      // A = Angle from center (positive = CW)

    Thread listener = new Thread(() -> {
      while(!Thread.interrupted()) {
        try {
          DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length); //create a new packet for the receiving data 
          serverSocket.receive(receivePacket); //receive the packet from the Socket
          // DriverStation.reportError("I GOT PI DATA", false);
          String segment = new String(receivePacket.getData()).replaceAll("\\s+",""); //remove whitespace and place data in 'segment'
          // DriverStation.reportWarning("new data: " + segment, false);
          latestSegment = segment.substring(segment.indexOf(":") + 1, segment.indexOf(";")); // store segment without borders
          latestTime = System.currentTimeMillis(); // add timestamp for stored segment
          // DriverStation.reportWarning("Seconds since update: " + getSecondsSinceUpdate(), false);
          String formattedString = segment.substring(segment.indexOf(":") + 1, segment.indexOf(";"));
          SmartDashboard.putString("RPi Data", formattedString); // put string on dashboard without borders
          latestData = analyzeData(formattedString);

        } catch (IOException e) { //thrown when the socket cannot receive the packet
          DriverStation.reportError("IO EXCEPTION", true);
        }
      }
    });
    
    listener.start();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Spotted", targetSpotted());
    SmartDashboard.putBoolean("Updated", getSecondsSinceUpdate() < 0.5);
  }

  /**
   * Prints dashboard indicators indicating whether the subsystem is ready for a match.
   * Indicators are to be used for pre-match only. They do not provide an accurite indication
   * of the state of a subsystem in mid match.
   * @return true if the system is ready for a match, false otherwise.
   */
  public boolean getSystemIsGo() {
    return getSecondsSinceUpdate() < 0.5;
  }

  /**
   * Retrieves the last known pixel coordinates of the target
   * @return [0] = X-coordinate (in pixels from left)
   *         [1] = Y-coordinate (in pixels from bottom)
   *         [2] = Distance (in inches)
   *         [3] = Angle from center (in degrees; positive = CW)
   *         {-1,-1,-1,-1} for no known location
   */
  public double[] getLatestData() {
    return latestData;
  }

  /**
   * Returns the width of the seen target in pixels, or -1 if no target is seen.
   */
  public double getTargetWidthPixels() {
    return latestData[2];
  }

  /**
   * Returns the height of the seen target in pixels, or -1 if no target is seen.
   */
  public double getTargetHeightPixels() {
    return latestData[3];
  }

  /**
   * Returns the distance of the camera to the target, or -1 if no target is seen.
   */
  public double getDistanceToTarget() {
    return latestData[4];
  }

  /**
   * Returns the horizontal angle (degrees) to the target, or 180 if no target is seen.
   */
  public double getHorizontalAngleToTarget() {
    return latestData[5];
  }

  /**
   * Returns the vertical angle (degrees) to the target, or 180 if no target is seen.
   */
  public double getVerticalAngleToTarget() {
    return latestData[6];
  }

  /**
   * Returns true if a target is seen, false otherwise.
   */
  public boolean targetSpotted() {
    return latestData[2] > -1;
  }

  /**
   * Returns the miliseconds since the pi sent the LastKnownLocation
   * @return ms since last received UDP packet
   */
  public double getSecondsSinceUpdate() {
    return Util.roundTo((double) ((System.currentTimeMillis() - latestTime) / 1000), 5);
  }

  /**
   * If data is being received, records whether or not its in "target lock" range
   * If dats is not being received, the last known state is kept
   */
  public void updateTargetLock(double[] data) {
    if (data[2] != -1) {
      inRange = data[2] < 0;
    }
  }

  /**
   * Gets the state of target lock
   * @return true if within range, false if out of range
   */
  public Boolean getWithinRange() {
    return inRange;
  }

  /**
   * Retrieves the last known pixel coordinates of the target
   * @return [0] = X-coordinate (in pixels from left)
   *         [1] = Y-coordinate (in pixels from bottom)
   *         [2] = Distance (in inches)
   *         [3] = Angle from center (in degrees; positive = CW)
   *         {-1,-1,-1,-1} for no known location
   */
  private double[] analyzeData(String input) {
    double[] newData = {-1, -1, -1, -1, -1, 180, 180};
    String[] stringData = input.split(",");

    if(stringData.length != newData.length) {
      DriverStation.reportWarning("INPUT STRING IMPROPERLY FORMATTED!", true);
      return newData;
    }

    try {
      for(int i=0; i<stringData.length; i++) {
        newData[i] = Integer.parseInt(stringData[i]);
      }
    } catch(Exception ex) {
      DriverStation.reportWarning("PARSING DATA ERROR: " + ex.getMessage(), true);
    }

    return newData;
  }
}