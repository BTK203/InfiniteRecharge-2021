/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Xbox;


public class SubsystemClimb extends SubsystemBase {
  /**
   * Creates a new SubsystemClimb.
   */
  private static CANSparkMax 
    scissors,
    winch;

  public SubsystemClimb() {
    scissors = new CANSparkMax(Constants.CLIMBER_SCISSOR_ID, MotorType.kBrushless);
    winch    = new CANSparkMax(Constants.CLIMBER_WINCH_ID, MotorType.kBrushless);
    configureMotors();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Scissor Position", scissors.getEncoder().getPosition());
  }

  public double ascendByController(Joystick controller) {
    double speed = Xbox.RIGHT_Y(controller);
    scissors.setIdleMode(IdleMode.kBrake);
    scissors.set(speed);
    return scissors.getOutputCurrent();
  }

  public double decendByController(Joystick controller) {
    double speed = Xbox.RT(controller) - Xbox.LT(controller);
    scissors.setIdleMode(IdleMode.kCoast);
    winch.set(speed);
    return winch.getOutputCurrent();
  }

  private void configureMotors() {
    scissors.setIdleMode(IdleMode.kBrake);
    scissors.setInverted(Constants.CLIMBER_SCISSOR_INVERT);

    winch.setIdleMode(IdleMode.kBrake);
    winch.setInverted(Constants.CLIMBER_WINCH_INVERT);
  }

  public double getPosition() {
    return scissors.getEncoder().getPosition();
  }
  
  public void setScissorPIDF(double p, double i, double d, double f, double lowLimit, double highLimit) {
    scissors.getPIDController().setP(p, 0);
    scissors.getPIDController().setI(i, 0);
    scissors.getPIDController().setD(d, 0);
    scissors.getPIDController().setFF(f, 0);
    scissors.getPIDController().setOutputRange(lowLimit, highLimit);
  }

  public void setScissorPosition(double position) {
    scissors.getPIDController().setReference(position, ControlType.kPosition);
  }
}
