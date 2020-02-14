/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Xbox;


public class SubsystemClimb extends SubsystemBase {
  /**
   * Creates a new SubsystemClimb.
   */
  private static CANSparkMax scissor;
  private static CANSparkMax winch;

  public SubsystemClimb() {
    scissor = new CANSparkMax(Constants.CLIMBER_SCISSOR_ID, MotorType.kBrushless);
    winch = new CANSparkMax(Constants.CLIMBER_WINCH_ID, MotorType.kBrushless);
  }

  
public void moveLiftByController(Joystick controller, Joystick joy) {
    scissor.set(Xbox.RIGHT_Y(controller) * Math.abs(Constants.scissorInhibitor));
    winch.set(Xbox.RIGHT_Y(joy) * Math.abs(Constants.winchInhibitor));
  }

  private double getAmperage(CANSparkMax motorController) {
    return motorController.getOutputCurrent();
  }

public double getScissorAmperage() {
  return getAmperage(scissor);
}

public double getWinchAmperage() {
  return getAmperage(winch);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
