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
import frc.robot.enumeration.ClimbPosition;
import frc.robot.Constants;
import frc.robot.util.Xbox;


public class SubsystemClimb extends SubsystemBase {
  private static CANSparkMax 
    scissors,
    winch;
  private static ClimbPosition storedPosition;

  /**
   * Creates a new SubsystemClimb.
   */
  public SubsystemClimb() {
    scissors = new CANSparkMax(Constants.CLIMBER_SCISSOR_ID, MotorType.kBrushless);
    winch    = new CANSparkMax(Constants.CLIMBER_WINCH_ID, MotorType.kBrushless);
    storedPosition = ClimbPosition.LOWEST;
    configureMotors();
  }

  /**
   * Runs with every robot frame.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Scissor Position", scissors.getEncoder().getPosition());
    SmartDashboard.putNumber("Winch Position", winch.getEncoder().getPosition());

    SmartDashboard.putNumber("Scissor Amps", scissors.getOutputCurrent());
    SmartDashboard.putNumber("Winch Amps", winch.getOutputCurrent());

    SmartDashboard.putNumber("Scissor Out", scissors.getAppliedOutput());
    SmartDashboard.putNumber("Winch Out", winch.getAppliedOutput());
  }

  /**
   * Moves the scissors based on input from the passed controller.
   * @param controller the controller to use to control the scissors.
   * @return current draw of the scissor motor.
   */
  public double moveScissorsByController(Joystick controller) {
    double speed = Xbox.LEFT_Y(controller);
    scissors.set(speed);
    return scissors.getOutputCurrent();
  }

  /**
   * Moves the winch based on input from the passed controller.
   * @param controller the controller to use to control the winch.
   * @return current draw of the winch motor.
   */
  public double moveWinchByController(Joystick controller) {
    double speed = Xbox.RT(controller) - Xbox.LT(controller);
    winch.set(speed);
    return winch.getOutputCurrent();
  }

  /**
   * Moves both the scissors and the winch with the passed controller.
   * @param controller the controller to use to control the climber.
   */
  public void driveByJoystick(Joystick controller) {
    moveScissorsByController(controller);
    moveWinchByController(controller);
  }

  /**
   * Configures braking modes and inverts on the motors.
   */
  private void configureMotors() {
    setScissorBraking(IdleMode.kCoast);
    scissors.setInverted(Constants.CLIMBER_SCISSOR_INVERT);

    winch.setIdleMode(IdleMode.kBrake);
    winch.setInverted(Constants.CLIMBER_WINCH_INVERT);
  }
  
  /**
   * Sets the encoder counts of the scissor and winch motors to 0.
   */
  public void zeroEncoders() {
    scissors.getEncoder().setPosition(0);
    winch.getEncoder().setPosition(0);
  }
  
  /**
   * Sets the percent output of the scissor motor.
   * @param speedz desired percent output (-1.0 to 1.0) of scissor motor.
   */
  public void setScissorsPercentOutput(double speedz) {
    scissors.set(speedz);
  }

  /**
   * Sets the percent output of the winch motor.
   * @param speedz desied percent output (-1.0 to 1.0) of the winch motor.
   */
  public void setWinchPercentOutput(double speedz) {
    winch.set(speedz);
  }

  /**
   * Sets the PID Constants of the scissor motor.
   * @param p P gain
   * @param i I gain
   * @param d D gain
   * @param f F gain
   * @param IZone Range from target at which I should take effect
   * @param lowLimit lowest allowable output (max: -1)
   * @param highLimit highest allowable output (max: 1)
   */
  public void setScissorPIDF(double p, double i, double d, double f, double IZone, double lowLimit, double highLimit) {
    scissors.getPIDController().setP(p, 0);
    scissors.getPIDController().setI(i, 0);
    scissors.getPIDController().setD(d, 0);
    scissors.getPIDController().setFF(f, 0);
    scissors.getPIDController().setIZone(IZone, 0);
    scissors.getPIDController().setOutputRange(lowLimit, highLimit);
  }

  /**
   * Sets the PID Constants of the winch motor
   * @param p P gain
   * @param i I gain
   * @param d D gain
   * @param f F gain
   * @param IZone Range from target at which the I gain takes effect
   * @param lowLimit lowest allowable output
   * @param highLimit highest allowable output
   */
  public void setWinchPIDF(double p, double i, double d, double f, double IZone, double lowLimit, double highLimit) {
    winch.getPIDController().setP(p);
    winch.getPIDController().setI(i);
    winch.getPIDController().setD(d);
    winch.getPIDController().setFF(f);
    winch.getPIDController().setIZone(IZone);
    winch.getPIDController().setOutputRange(lowLimit, highLimit);
  }

  /**
   * Sets the target position (in rotations) of the scissor motor.
   * @param position
   */
  public void setScissorsPosition(double position) {
    scissors.getPIDController().setReference(position, ControlType.kPosition);
  }

  /**
   * Sets the target position (in rotations) of the winch motor.
   * @param position the new target position of the scissor motor
   */
  public void setWinchPosition(double position) {
    winch.getPIDController().setReference(position, ControlType.kPosition);
  }

  /**
   * Returns the current position of the scissor motor in rotations.
   */
  public double getScissorPosition() {
    return scissors.getEncoder().getPosition();
  }

  /**
   * Returns the current position of the winch motor in rotations.
   */
  public double getWinchPosition() {
    return winch.getEncoder().getPosition();
  }

  /**
   * Sets the braking mode of the scissors.
   * @param mode IdleMode of the scissors. Either kCoast or kBrake.
   */
  public void setScissorBraking(IdleMode mode) {
    scissors.setIdleMode(mode);
  }

  /**
   * Sets the target position of the climber.
   * @param position ClimbPosition describing where the climber should be.
   */
  public void setStoredPosition(ClimbPosition position) {
    storedPosition = position;
  }

  /**
   * Returns the current target position of the climber.
   */
  public ClimbPosition getStoredPosition() {
    return storedPosition;
  }
}
