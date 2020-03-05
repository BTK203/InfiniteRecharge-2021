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

  
  public double getVelocity() {
    return turretFlywheel.getEncoder().getVelocity() * Constants.FLYWHEEL_GEAR_RATIO;
  }

  public void setPIDF(double p, double i, double d, double f, double lowLimit, double highLimit, double izone) {
    turretFlywheel.getPIDController().setP(p, 0);
    turretFlywheel.getPIDController().setI(i, 0);
    turretFlywheel.getPIDController().setD(d, 0);
    turretFlywheel.getPIDController().setFF(f, 0);
    turretFlywheel.getPIDController().setOutputRange(lowLimit, highLimit);

    turretFlywheel.getPIDController().setIZone(izone, 0);
  }

  public void setVelocity(double velocity) {
    turretFlywheel.getPIDController().setReference(velocity, ControlType.kVelocity);
  }

  private void configureMotor() {
    turretFlywheel.setIdleMode(IdleMode.kCoast);
    turretFlywheel.setInverted(Constants.TURRET_FLYWHEEL_INVERT);
    turretFlywheel.setSmartCurrentLimit(Constants.FLYWHEEL_AMP_LIMIT);
  }
}
