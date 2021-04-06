/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Util;
import frc.robot.util.Xbox;

/**
 * Turret pitch and yaw
 */
public class SubsystemTurret extends SubsystemBase {
 
  private TalonSRX 
    turretYaw,
    turretPitch;

  private int
    totalYawTicks,
    totalPitchTicks;

  private boolean 
    pitchPositioningDisabled;

  /**
   * Creates a new SubsystemTurret.
   */
  public SubsystemTurret() {
    turretYaw = new TalonSRX(Constants.TURRET_YAW_ID);
    turretPitch = new TalonSRX(Constants.TURRET_PITCH_ID);

    totalYawTicks = Constants.DEFAULT_TURRET_YAW_TICKS;
    totalPitchTicks = Constants.DEFAULT_TURRET_PITCH_TICKS;

    pitchPositioningDisabled = false;

    configureMotors();
  }

  /**
   * Runs with every robot frame.
   */
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

    SmartDashboard.putNumber("Yaw Out", turretYaw.getMotorOutputPercent());
    SmartDashboard.putNumber("Pitch Out", turretPitch.getMotorOutputPercent());
  
    if(getYawLeftLimit()) {
      turretYaw.getSensorCollection().setQuadraturePosition(0, 0);
    }

    if(getPitchLowerLimit()) {
      turretPitch.getSensorCollection().setQuadraturePosition(0, 0);
    }    
  }

  /**
   * Prints dashboard indicators indicating whether the subsystem is ready for a match.
   * Indicators are to be used for pre-match only. They do not provide an accurite indication
   * of the state of a subsystem in mid match.
   * @return true if the system is ready for a match, false otherwise.
   */
  public boolean getSystemIsGo() {
    boolean yawConnected = turretYaw.getBusVoltage() > Constants.SPARK_MINIMUM_VOLTAGE;
    boolean pitchConnected = turretPitch.getBusVoltage() > Constants.SPARK_MINIMUM_VOLTAGE;

    SmartDashboard.putBoolean("Yaw Connected", yawConnected);
    SmartDashboard.putBoolean("Pitch Connected", pitchConnected);

    return yawConnected && pitchConnected;
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

    speedx *= Constants.TURRET_YAW_ABSOLUTE_MAX_OUTPUT;

    speedx = speedx * Util.getAndSetDouble("Turret Spin Inhibitor Yaw", 0.7);
    speedy = speedy * Util.getAndSetDouble("Turret Spin Inhibitor Pitch", 1);

    speedx = (speedx < -1 * Constants.TURRET_YAW_ABSOLUTE_MAX_OUTPUT ? -1 * Constants.TURRET_YAW_ABSOLUTE_MAX_OUTPUT : (speedx > Constants.TURRET_YAW_ABSOLUTE_MAX_OUTPUT ? Constants.TURRET_YAW_ABSOLUTE_MAX_OUTPUT : speedx));

    turretYaw.set(ControlMode.PercentOutput, speedx);
    turretPitch.set(ControlMode.PercentOutput, speedy);
  }

  /**
   * Sets the PIDF constants of the turret motor.
   * @param p desired P gain
   * @param i desired I gain
   * @param d desired D gain
   * @param f desired F gain
   * @param highOut maximum output percent
   * @param izone proximity to target at which I gain starts to take effect
   */
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

  /**
   * Sets the PIDF constants of the pitch motor.
   * @param p desired P gain
   * @param i desired I gain
   * @param d desired D gain
   * @param f desired F gain
   * @param highOut maximum output percent
   * @param izone proximity to target at which I gain starts to take effect
   */
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

  /**
   * Sets the target yaw position
   * @param position target yaw position in ticks.
   */
  public void setYawPosition(double position) {
    turretYaw.set(ControlMode.Position, position);

    SmartDashboard.putNumber("Yaw PID Target", position);
    SmartDashboard.putNumber("Yaw PID Error", Math.abs(turretYaw.getSensorCollection().getQuadraturePosition()) - position);
  }

  /**
   * Sets the target pitch position
   * @param position target pitch position in ticks.
   */
  public void setPitchPosition(double position) {
    if(!pitchPositioningDisabled) {
      turretPitch.set(ControlMode.Position, position);

      SmartDashboard.putNumber("Pitch PID Target", position);
      SmartDashboard.putNumber("Pitch PID Error", turretPitch.getSensorCollection().getQuadraturePosition() - position);
    }
  }

  /**
   * Sets the percent output of the yaw motor.
   * @param percent percent output 
   */
  public void setYawPercentOutput(double percent) {
    double inhibited = percent * Constants.TURRET_YAW_ABSOLUTE_MAX_OUTPUT;
    inhibited *= Util.getAndSetDouble("Turret Spin Inhibitor Yaw", 0.7);
    turretYaw.set(ControlMode.PercentOutput, inhibited);
  }

  /**
   * Sets the percent output of the pitch motor.
   * @param percent percent output
   */
  public void setPitchPercentOutput(double percent) {
      turretPitch.set(ControlMode.PercentOutput, percent);
  }

  public void setPitchPositioningDisabled(boolean disabled) {
    pitchPositioningDisabled = disabled;
  }
  
  /**
   * Returns the current position of the yaw motor in ticks.
   */
  public double getYawPosition() {
    return turretYaw.getSensorCollection().getQuadraturePosition();
  }

  /**
   * Returns the current position of pitch motor in ticks.
   */
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
   * NOTE: Left limit is the "high" limit.
   */
  public boolean getYawLeftLimit() {
    return turretYaw.isRevLimitSwitchClosed() == 1;
  }

  /**
   * Returns true if the pitch motor's lower limit is closed, false otherwise.
   */
  public boolean getPitchLowerLimit() {
    return turretPitch.isFwdLimitSwitchClosed() == 1;
  }

  /**
   * Returns true if the pitch motor's upper limit is closed, false otherwise.
   */
  public boolean getPitchUpperLimit() {
    return turretPitch.isRevLimitSwitchClosed() == 1;
  }

  /**
   * Sets the current yaw encoder position.
   * @param newPosition the new current yaw position, in encoder ticks.
   */
  public void setCurrentYawEncoderPosition(int newPosition) {
    turretYaw.getSensorCollection().setQuadraturePosition(newPosition, 0);
  }

  /**
   * Sets the current pitch encoder position.
   * @param newPosition the new current pitch position, in encoder ticks.
   */
  public void setCurrentPitchEncoderPosition(int newPosition) {
    turretPitch.getSensorCollection().setQuadraturePosition(newPosition, 0);
  }

  /**
   * Attempts to set the total yaw ticks by hitting the right limit.
   * NOTE: Motor percent output must be set elsewhere. This method only checks the limit switch
   * and sets the encoder position if it is closed.
   * @return true if the total tick count was successfully set, false otherwise.
   */
  public boolean attemptToSetTotalYawTicks() {
    if(getYawRightlimit()) {
      this.totalYawTicks = turretYaw.getSensorCollection().getQuadraturePosition();
      return true;
    }

    return false;
  }

  /**
   * Attempts to set the total pitch ticks by hitting the upper limit.
   * NOTE: Just like attemptToSetTotalYawTicks(), this method does not
   * set the motor percent output. That must be called along with this method.
   * @return true if the total pitch tick count was successfully set, false otherwise.
   */
  public boolean attemptToSetTotalPitchTicks() {
    if(getPitchUpperLimit()) {
      this.totalPitchTicks = turretPitch.getSensorCollection().getQuadraturePosition();
      return true;
    }

    return false;
  }

  /**
   * Returns the total tick count of the yaw encoder.
   */
  public double getTotalYawTicks() {
    return totalYawTicks;
  }

  /**
   * Returns the total tick count of the pitch encoder.
   * @return
   */
  public double getTotalPitchTicks() {
    return totalPitchTicks;
  }

  /**
   * Sets output inverts, encoder inverts, and neutral modes of the motors.
   */
  private void configureMotors() {
    turretPitch.setNeutralMode(NeutralMode.Brake);
    turretYaw.setNeutralMode(NeutralMode.Brake);

    turretPitch.setInverted(Constants.TURRET_PITCH_INVERT);
    turretYaw.setInverted(Constants.TURRET_YAW_INVERT);

    turretYaw.configContinuousCurrentLimit(Constants.TURRET_YAW_AMP_LIMIT);

    turretYaw.setSensorPhase(true);
    turretPitch.setSensorPhase(true);
  }
}
