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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The spinny one
 */
public class SubsystemFlywheel extends SubsystemBase {
  private CANSparkMax
    turretFlywheel;

  /**
   * Creates a new SubsystemturretFlywheel.
   */
  public SubsystemFlywheel() {
    turretFlywheel = new CANSparkMax(Constants.TURRET_FLYWHEEL_ID, MotorType.kBrushless);
    configureMotor();
  }

  /**
   * Runs with every robot frame.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double motorVelocity = turretFlywheel.getEncoder().getVelocity();
    SmartDashboard.putNumber("FW Motor Velocity", motorVelocity);
    SmartDashboard.putNumber("FW Wheel Velocity", motorVelocity * Constants.FLYWHEEL_GEAR_RATIO);

    SmartDashboard.putNumber("FW Output", turretFlywheel.getAppliedOutput());
    SmartDashboard.putNumber("FW Amps", turretFlywheel.getOutputCurrent());
  }

  /**
   * Set the turretFlywheel speed
   * @param speedz The percent to drive (-1 to 1)
   */
  public void setFlywheelPercentOutput(double speedz){
    turretFlywheel.set(speedz);
  }

  /**
   * Returns the velocity (RPM) of the FLYWHEEL, NOT the motor.
   */
  public double getVelocity() {
    return turretFlywheel.getEncoder().getVelocity() * Constants.FLYWHEEL_GEAR_RATIO;
  }

  /**
   * Sets the PIDF constants of the flywheel motor.
   * @param p desired P gain
   * @param i desired I gain
   * @param d desired D gain
   * @param f desired F gain
   * @param lowLimit lowest allowable output
   * @param highLimit highest allowable output
   * @param izone proximity to target at which I gain takes effect
   */
  public void setPIDF(double p, double i, double d, double f, double lowLimit, double highLimit, double izone) {
    turretFlywheel.getPIDController().setP(p, 0);
    turretFlywheel.getPIDController().setI(i, 0);
    turretFlywheel.getPIDController().setD(d, 0);
    turretFlywheel.getPIDController().setFF(f, 0);
    turretFlywheel.getPIDController().setOutputRange(lowLimit, highLimit);

    turretFlywheel.getPIDController().setIZone(izone, 0);
  }

  /**
   * Sets the target velocity (RPM) of the MOTOR, NOT the flywheel
   * @param velocity
   */
  public void setVelocity(double velocity) {
    turretFlywheel.getPIDController().setReference(velocity, ControlType.kVelocity);
  }

  /**
   * Configures the IdleMode, invert, and amp limit of the motor.
   */
  private void configureMotor() {
    turretFlywheel.setIdleMode(IdleMode.kCoast);
    turretFlywheel.setInverted(Constants.TURRET_FLYWHEEL_INVERT);
    turretFlywheel.setSmartCurrentLimit(Constants.FLYWHEEL_AMP_LIMIT);
  }
}
