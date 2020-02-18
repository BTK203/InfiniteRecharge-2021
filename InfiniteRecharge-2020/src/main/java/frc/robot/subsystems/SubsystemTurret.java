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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

  public SubsystemTurret() {
    turretYaw = new TalonSRX(Constants.TURRET_YAW_ID);
    turretPitch = new TalonSRX(Constants.TURRET_PITCH_ID);

    configureMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Yaw Position", turretYaw.getSensorCollection().getQuadraturePosition());
    SmartDashboard.putNumber("Pitch Position", turretPitch.getSensorCollection().getQuadraturePosition());

    SmartDashboard.putNumber("Yaw Forward Limit", turretYaw.isFwdLimitSwitchClosed());
    SmartDashboard.putNumber("Yaw Backward Limit", turretYaw.isRevLimitSwitchClosed());

    if(turretPitch.isFwdLimitSwitchClosed() > 0) {
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

  public void setYawPIDF(double p, double i, double d, double f, double highOut) {
    turretYaw.config_kP(0, p);
    turretYaw.config_kI(0, i);
    turretYaw.config_kD(0, d);
    turretYaw.config_kF(0, f);
    turretYaw.configClosedLoopPeakOutput(0, highOut);
  }

  public void setPitchPIDF(double p, double i, double d, double f, double highOut) {
    turretPitch.config_kP(0, p);
    turretPitch.config_kI(0, i);
    turretPitch.config_kD(0, d);
    turretPitch.config_kF(0, f);
    turretPitch.configClosedLoopPeakOutput(0, highOut);
  }

  public void setYawPosition(double position) {
    turretYaw.set(ControlMode.Position, position);
  }

  public void setPitchPosition(double position) {
    turretPitch.set(ControlMode.Position, position);
  }
  
  public double getYawPosition() {
    return turretYaw.getSensorCollection().getQuadraturePosition();
  }

  public double getPitchPosition() {
    return turretPitch.getSensorCollection().getQuadraturePosition();
  }

  private void configureMotors() {
    turretPitch.setNeutralMode(NeutralMode.Brake);
    turretYaw.setNeutralMode(NeutralMode.Brake);
  }
}
