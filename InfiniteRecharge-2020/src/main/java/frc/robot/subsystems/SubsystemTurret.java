/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Util;
import frc.robot.util.Xbox;

public class SubsystemTurret extends SubsystemBase {
  /**
   * Creates a new SubsystemTurret.
   */
  private TalonSRX 
    turretYaw,
    turretPitch;

  private int
    totalYawTicks,
    totalPitchTicks;

  public SubsystemTurret() {
    turretYaw = new TalonSRX(Constants.TURRET_YAW_ID);
    turretPitch = new TalonSRX(Constants.TURRET_PITCH_ID);

    totalYawTicks = Constants.DEFAULT_TURRET_YAW_TICKS;
    totalPitchTicks = Constants.DEFAULT_TURRET_PITCH_TICKS;

    configureMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Yaw Position", turretYaw.getSensorCollection().getQuadraturePosition());
    SmartDashboard.putNumber("Pitch Position", turretPitch.getSensorCollection().getQuadraturePosition());

    SmartDashboard.putBoolean("Yaw Forward Limit", turretYaw.isFwdLimitSwitchClosed() == 1);
    SmartDashboard.putBoolean("Yaw Backward Limit", turretYaw.isRevLimitSwitchClosed() == 1);

    SmartDashboard.putBoolean("Pitch Forward Limit", turretPitch.isFwdLimitSwitchClosed() == 1);
    SmartDashboard.putBoolean("Pitch Backward Limit", turretPitch.isRevLimitSwitchClosed() == 1);

    SmartDashboard.putNumber("Yaw Ticks", totalYawTicks);
    SmartDashboard.putNumber("Pitch Ticks", totalPitchTicks);

    SmartDashboard.putNumber("Yaw Amps", turretYaw.getStatorCurrent());
    SmartDashboard.putNumber("Pitch Amps", turretPitch.getStatorCurrent());

    if(getYawLeftLimit()) {
      turretYaw.getSensorCollection().setQuadraturePosition(0, 0);
    }

    if(getPitchLowerLimit()) {
      turretPitch.getSensorCollection().setQuadraturePosition(0, 0);
    }    
  }

  /**
   * Move the turret
   * @param controller The controller to use.
   */
  public void moveTurret(Joystick controller) {
    double speedx;
    double speedy;

    speedx = Xbox.LEFT_X(controller);
    speedy = Xbox.RIGHT_Y(controller);

    speedx = speedx * Util.getAndSetDouble("Turret Spin Inhibitor Yaw", 1);
    speedy = speedy * Util.getAndSetDouble("Turret Spin Inhibitor Pitch", 1);

    turretYaw.set(ControlMode.PercentOutput, speedx);
    turretPitch.set(ControlMode.PercentOutput, speedy);
  }

  public void setYawPIDF(double p, double i, double d, double f, double highOut, int izone) {

    turretYaw.config_kP(0, p);
    turretYaw.config_kI(0, i);
    turretYaw.config_IntegralZone(0, izone);
    turretYaw.config_kD(0, d);
    turretYaw.config_kF(0, f);

    turretYaw.configPeakOutputForward(highOut);
    turretYaw.configPeakOutputReverse(highOut * -1);
    turretYaw.configAllowableClosedloopError(0, 0, 0);
  }

  public void setPitchPIDF(double p, double i, double d, double f, double highOut, int izone) {
    turretPitch.config_kP(0, p);
    turretPitch.config_kI(0, i);
    turretYaw.config_IntegralZone(0, izone);
    turretPitch.config_kD(0, d);
    turretPitch.config_kF(0, f);

    turretPitch.configPeakOutputForward(highOut, 0);
    turretPitch.configPeakOutputReverse(highOut * -1, 0);
    turretPitch.configAllowableClosedloopError(0, 0, 0);
  }

  public void setYawPosition(double position) {
    turretYaw.set(ControlMode.Position, position);

    turretYaw.setSensorPhase(false);

    SmartDashboard.putNumber("Yaw PID Target", position);
    SmartDashboard.putNumber("Yaw PID Error", Math.abs(turretYaw.getSensorCollection().getQuadraturePosition()) - position);

    SmartDashboard.putNumber("Yaw Amps", turretYaw.getStatorCurrent());
    SmartDashboard.putNumber("Pitch Amps", turretPitch.getStatorCurrent());
  }

  public void setPitchPosition(double position) {
    turretPitch.set(ControlMode.Position, position);
  }

  public void setYawPercentOutput(double percent) {
    turretYaw.set(ControlMode.PercentOutput, percent);
  }

  public void setPitchPercentOutput(double percent) {
    turretPitch.set(ControlMode.PercentOutput, percent);
  }
  
  public double getYawPosition() {
    return turretYaw.getSensorCollection().getQuadraturePosition();
  }

  public double getPitchPosition() {
    return turretPitch.getSensorCollection().getQuadraturePosition();
  }

  /**
   * Returns the Yaw motors right limit switches state (true for closed). 
   * NOTE: Right Limit is also the "zero" limit.
   */
  public boolean getYawRightlimit() {
    return turretYaw.isFwdLimitSwitchClosed() == 1;
  }

  /**
   * Returns the Yaw motors left limit switches state(true for closed).
   * NOTE Left limit is the "high" limit.
   */
  public boolean getYawLeftLimit() {
    return turretYaw.isRevLimitSwitchClosed() == 1;
  }

  public boolean getPitchLowerLimit() {
    return turretPitch.isFwdLimitSwitchClosed() == 1;
  }

  public boolean getPitchUpperLimit() {
    return turretPitch.isRevLimitSwitchClosed() == 1;
  }

  public void setCurrentYawEncoderPosition(int newPosition) {
    turretYaw.getSensorCollection().setQuadraturePosition(newPosition, 0);
  }

  public void setCurrentPitchEncoderPosition(int newPosition) {
    turretPitch.getSensorCollection().setQuadraturePosition(newPosition, 0);
  }

  public boolean attemptToSetTotalYawTicks() {
    if(getYawRightlimit()) {
      this.totalYawTicks = turretYaw.getSensorCollection().getQuadraturePosition();
      return true;
    }

    return false;
  }

  public boolean attemptToSetTotalPitchTicks() {
    if(getPitchUpperLimit()) {
      this.totalPitchTicks = turretPitch.getSensorCollection().getQuadraturePosition();
      return true;
    }

    return false;
  }

  public double getTotalYawTicks() {
    return totalYawTicks;
  }

  public double getTotalPitchTicks() {
    return totalPitchTicks;
  }

  private void configureMotors() {
    turretPitch.setNeutralMode(NeutralMode.Brake);
    turretYaw.setNeutralMode(NeutralMode.Brake);
  }
}
