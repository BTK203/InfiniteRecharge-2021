// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubsystemJevois extends SubsystemBase {
  SerialPort port;
  private int
    closestBallX,
    closestBallY,
    closestBallRadius;

  private String 
    currentMessage,
    lastCompletedMessage;

  /** Creates a new SubsystemJevois. */
  public SubsystemJevois() {
    port = new SerialPort(Constants.JEVOIS_BAUD_RATE, Constants.JEVOIS_PORT);
  }

  //[x,y,r],[x,y,r]
  @Override
  public void periodic() {
    currentMessage += port.readString();

    //parse USB message
  }
}
