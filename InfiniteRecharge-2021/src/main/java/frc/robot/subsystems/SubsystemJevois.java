// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PowerCell;

public class SubsystemJevois extends SubsystemBase {
  private DatagramSocket jevois;
  private ArrayList<PowerCell> powerCells;
  private int powerCellsSpotted;
  private String lastCompletedMessage;
  private long lastUpdatedTime;

  /** Creates a new SubsystemJevois. */
  public SubsystemJevois() {
    powerCells = new ArrayList<PowerCell>();
    powerCellsSpotted = 0;
    lastCompletedMessage = "No Message!";
    lastUpdatedTime = 0;

    try {
      jevois = new DatagramSocket(Constants.JEVOIS_PORT);

      //run serial and parsing code in a separate thread
      new Thread(
        () -> {
          while(true) {
            try {
              DatagramPacket packet = new DatagramPacket(new byte[Constants.JEVOIS_BYTES], Constants.JEVOIS_BYTES);
              jevois.receive(packet);
              String incomingData = new String(packet.getData());
              lastUpdatedTime = System.currentTimeMillis();
              update(incomingData);
            } catch(IOException ex) {
              DriverStation.reportError("Subsystem Jevois IOException", false);
            }
          }
        }
      ).start();
    } catch(SocketException ex) {
      DriverStation.reportError("Socket Exception", true);
      DriverStation.reportError(ex.getMessage(), false);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Jevois Data", lastCompletedMessage);
    SmartDashboard.putNumber("Power cells spotted", powerCellsSpotted);
    SmartDashboard.putBoolean("Jevois Updated", updated());

    //list y-coordinates of power cells in increasing order
    String powerCellsString = "";
    for(int i=0; i<powerCells.size(); i++) {
      powerCellsString += powerCells.get(i).toString();
      if(i < powerCells.size() - 1) {
        powerCellsString += ", ";
      }
    }
    SmartDashboard.putString("Power Cells", powerCellsString);
  }

  /**
   * Returns whether or not the Jevois has spotted a ball.
   * @return True if a ball is spotted, false otherwise.
   */
  public boolean ballSpotted() {
    return powerCellsSpotted > 0;
  }

  /**
   * Returns the x-coordinate of the closest ball.
   * A return value of 0 indicates that the ball is exactly in the center of the screen,
   * a negative value indicates that the ball is left of center, and a positive value indicates 
   * that the ball is right of center.
   * @return The x coordinate of the closeset ball in pixels.
   */
  public int getHorizontalPosition() {
    return powerCells.get(0).getX();
  }

  /**
   * Returns an array of PowerCells that are listed in order of closest to furthest.
   * @return Some PowerCells.
   */
  public ArrayList<PowerCell> getPowerCells() {
    return powerCells;
  }

  /**
   * Returns whether or not data has recently been received from the Jevois.
   * @return True if data has been recently received, false otherwise.
   */
  public boolean updated() {
    return System.currentTimeMillis() - lastUpdatedTime < 250;
  }

  /**
   * Parses incoming data into PowerCells.
   * @param segments Formatted segments of data.
   */
  private void parseData(String[] segments) {
    ArrayList<PowerCell> powerCellsList = new ArrayList<PowerCell>();
    for(String segment : segments) {
      if(!segment.isEmpty()) {
        segment = segment.substring(segment.indexOf("[") + 1);

        String[] data = segment.split(",");
        try {
          int x = (int) Double.parseDouble(data[0]);
          int y = (int) Double.parseDouble(data[1]);
          int radius = (int) Double.parseDouble(data[2]);

          int centeredX = x - (Constants.JEVOIS_RESOLUTION_X / 2);

          //create a PowerCell and put it into the list
          PowerCell newPowerCell = new PowerCell(centeredX, y, radius);

          if(powerCellsList.size() == 0 || powerCellsList.get(0).getY() < newPowerCell.getY()) {
            powerCellsList.add(0, newPowerCell);
            continue;
          }

          //do insertion sort
          boolean inserted = false;
          for(int i=1; i<powerCellsList.size(); i++) {
            if(powerCellsList.get(i - 1).getY() >= newPowerCell.getY() && powerCellsList.get(i).getY() <= newPowerCell.getY()) {
              powerCellsList.add(i, newPowerCell);
              inserted = true;
              break;
            }
          }

          if(!inserted) {
            powerCellsList.add(newPowerCell);
          }

        } catch(NumberFormatException ex) {
          DriverStation.reportError("SubsystemJevois could not parse data!", true);
        }
      } 
    }

    powerCellsSpotted = powerCellsList.size();
    powerCells = powerCellsList;
  }

  /**
   * Updates powerCells based on the incoming data from the jevois
   * @param incoming The most recent data on the jevois buffer.
   */
  private void update(String incoming) {
    incoming = incoming.substring(0, incoming.indexOf(";"));
    
    if(incoming.contains("]")) {
      String[] segments = incoming.split("]");
      parseData(segments);
    } else {
      powerCellsSpotted = 0;
    }    
  }

  /**
   * TEST METHODS
   */

  private boolean testSort(String[] segments) {
    String str = "";
    for(String a : segments) {
      str += a + ", ";
    }

    System.out.println("Organizing " + str);
    parseData(segments);
    
    String str2 = "";
    boolean decreasing = true;
    for(int i=0; i<powerCells.size(); i++) {
      str2 += Integer.valueOf(powerCells.get(i).getY()).toString();
      
      if(i < powerCells.size() - 1) {
        if(powerCells.get(i).getY() < powerCells.get(i + 1).getY()) {
          decreasing = false;
        }

        str2 += ", ";
      }
    }

    System.out.println("Result: " + str2);
    System.out.println("Decreasing: " + (decreasing ? "YES" : "NO"));
    return decreasing;
  }


  public boolean test() {
    boolean success = true;
    for(int i=0; i<5; i++) {
      int numYs = (int) (Math.random() * 11) +5; //generate anywhere from 5 to 15 ys
      String[] segments = new String[numYs];
      for(int n=0; n<numYs; n++) {
        int y = (int) (Math.random() * 5);
        String segment = "[0," + Integer.valueOf(y).toString() + ",0";
        segments[n] = segment;
      }

      success = success && testSort(segments);
    }

    return success;
  }
}
