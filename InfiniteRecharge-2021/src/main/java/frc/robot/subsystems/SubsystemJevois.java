// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubsystemJevois extends SubsystemBase {
  private SerialPort port;
  private int
    closestBallX,
    closestBallY,
    closestBallRadius;

  private String 
    currentMessage,
    lastCompletedMessage;

  private boolean portInitalized;
  private long lastUpdatedTime;

  /** Creates a new SubsystemJevois. */
  public SubsystemJevois() {
    closestBallX = -1;
    closestBallY = -1;
    closestBallRadius = -1;
    currentMessage = "";
    lastCompletedMessage = "No Message!";
    portInitalized = false;
    lastUpdatedTime = 0;

    try {
      port = new SerialPort(Constants.JEVOIS_BAUD_RATE, Constants.JEVOIS_PORT);
      portInitalized = true;

      //run serial and parsing code in a separate thread
      new Thread(
        () -> {
          while(true) {
            if(portInitalized) {
              currentMessage += port.readString();
            }
        
            int newline = currentMessage.lastIndexOf("\n");
            if(newline >= 0) {
              lastCompletedMessage = currentMessage.substring(0, newline);
              currentMessage = currentMessage.substring(newline + 1);
              
              lastUpdatedTime = System.currentTimeMillis();
        
              if(!lastCompletedMessage.contains("None")) {
                String[] segments = lastCompletedMessage.split("]");
                int closestX = Integer.MAX_VALUE;
                int closestY = 0;
                int closestRadius = 0;
                for(String segment : segments) {
                  if(!segment.isEmpty()) {
                    segment = segment.substring(segment.indexOf("[") + 1);

                    String[] data = segment.split(",");
                    try {
                      int x = (int) Double.parseDouble(data[0]);
                      int y = (int) Double.parseDouble(data[1]);
                      int radius = (int) Double.parseDouble(data[2]);
          
                      int centeredX = x - (Constants.JEVOIS_RESOLUTION_X / 2);
          
                      if(Math.abs(centeredX) < Math.abs(closestX)) {
                        closestX = centeredX;
                        closestY = y;
                        closestRadius = radius;
                      }
                    } catch(NumberFormatException ex) {
                      // DriverStation.reportError("SubsystemJevois could not parse data!", true); //TODO either fix this or delete it forever
                    }
                  } 
                }

                closestBallX = closestX;
                closestBallY = closestY;
                closestBallRadius = closestRadius;
              } else {
                closestBallX = -9999;
                closestBallY = -1;
                closestBallRadius = -1;
              }
            }
        
            if(!portInitalized) {
              DriverStation.reportWarning("Jevois Port not initalized! Loop will not run", false);
            }        
          }
        }
      ).start();
    } catch(UncleanStatusException ex) {
      DriverStation.reportError("Unclean Status! Is the right Port specified?", true);
      DriverStation.reportError(ex.getMessage(), false);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Jevois Data", lastCompletedMessage);
    SmartDashboard.putNumber("Jevois X", closestBallX);
    SmartDashboard.putNumber("Jevois Y", closestBallY);
    SmartDashboard.putNumber("Jevois Radius", closestBallRadius);
    SmartDashboard.putBoolean("Jevois Updated", updated());
    SmartDashboard.putString("Jevois current message", currentMessage);
  }

  /**
   * Returns whether or not the Jevois has spotted a ball.
   * @return True if a ball is spotted, false otherwise.
   */
  public boolean ballSpotted() {
    return closestBallRadius > -1;
  }

  /**
   * Returns the x-coordinate of the closest ball.
   * A return value of 0 indicates that the ball is exactly in the center of the screen,
   * a negative value indicates that the ball is left of center, and a positive value indicates 
   * that the ball is right of center.
   * @return The x coordinate of the closeset ball in pixels.
   */
  public int getHorizontalPosition() {
    return closestBallX;
  }

  /**
   * Returns whether or not data has recently been received from the Jevois.
   * @return True if data has been recently received, false otherwise.
   */
  public boolean updated() {
    return System.currentTimeMillis() - lastUpdatedTime < 250;
  }
}
