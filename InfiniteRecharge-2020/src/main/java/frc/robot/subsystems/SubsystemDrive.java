/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;



import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Util;
import frc.robot.util.Xbox;

public class SubsystemDrive extends SubsystemBase {
  private static CANSparkMax leftMaster;
  private static CANSparkMax leftSlave;
  private static CANSparkMax rightMaster;
  private static CANSparkMax rightSlave;

  /**
   * Creates a new SubsystemDrive.
   */
  public SubsystemDrive() {
    leftMaster = new CANSparkMax(Constants.DRIVE_LEFT_MASTER_ID, MotorType.kBrushless);
    leftSlave = new CANSparkMax(Constants.DRIVE_LEFT_SLAVE_ID, MotorType.kBrushless);
    rightMaster = new CANSparkMax(Constants.DRIVE_RIGHT_MASTER_ID, MotorType.kBrushless);
    rightSlave = new CANSparkMax(Constants.DRIVE_RIGHT_SLAVE_ID, MotorType.kBrushless);

    setBraking();
    setRamps();
    setFollowers();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Right Position", rightMaster.getEncoder().getPosition());
    SmartDashboard.putNumber("Left Position", leftMaster.getEncoder().getPosition());

    SmartDashboard.putNumber("Right Output", rightMaster.getAppliedOutput());
    SmartDashboard.putNumber("Left Output", leftMaster.getAppliedOutput());

    SmartDashboard.putNumber("Right Amps", rightMaster.getOutputCurrent());
    SmartDashboard.putNumber("Left Amps", leftMaster.getOutputCurrent());
  }

  /**
   * Drives the drivetrain motors using the passed controller
   * @param controller The controller to drive with
   */
  public void DriveTankByController(Joystick controller) {
    setInverts();

    double throttle = Xbox.RT(controller) - Xbox.LT(controller); 
    double steering = Xbox.LEFT_X(controller);

    double driveRight = throttle + steering;
    double driveLeft = throttle - steering; 

    driveRight = (driveRight < -1 ? -1 : (driveRight > 1 ? 1 : driveRight));
    driveLeft = (driveLeft < -1 ? -1 : (driveLeft > 1 ? 1 : driveLeft));

    leftMaster.set(driveLeft);
    leftSlave.set(driveLeft);
    rightMaster.set(driveRight);
    rightSlave.set(driveRight);
  }

  public void setPercentOutput(double percentOutput) {
    rightMaster.set(percentOutput);
    rightSlave.set(percentOutput);
    leftMaster.set(percentOutput);
    leftSlave.set(percentOutput);
  }

  public void setLeftPosition(double leftPosition) {
    leftMaster.getPIDController().setReference(leftPosition, ControlType.kPosition);
  }

  public void setRightPosition(double rightPosition) {
    rightMaster.getPIDController().setReference(rightPosition, ControlType.kPosition);
  }

  public double getLeftPosition() {
    return leftMaster.getEncoder().getPosition();
  }

  public double getRightPosition() {
    return rightMaster.getEncoder().getPosition();
  }

  public void setPIDConstants(double kP, double kI, double kD, double kF, double iZone, double outLimit) {
    leftMaster.getPIDController().setP(kP);
    leftMaster.getPIDController().setI(kI);
    leftMaster.getPIDController().setD(kD);
    leftMaster.getPIDController().setFF(kF);
    leftMaster.getPIDController().setIZone(iZone);
    leftMaster.getPIDController().setOutputRange(outLimit * -1, outLimit);

    rightMaster.getPIDController().setP(kP);
    rightMaster.getPIDController().setI(kI);
    rightMaster.getPIDController().setD(kD);
    rightMaster.getPIDController().setFF(kF);
    rightMaster.getPIDController().setIZone(iZone);
    rightMaster.getPIDController().setOutputRange(outLimit * -1, outLimit);
  }

  private void setInverts() {
    leftMaster.setInverted(Constants.DRIVE_LEFT_MASTER_INVERT);
    leftSlave.setInverted(Constants.DRIVE_LEFT_SLAVE_INVERT);
    rightMaster.setInverted(Constants.DRIVE_RIGHT_MASTER_INVERT);
    rightSlave.setInverted(Constants.DRIVE_RIGHT_SLAVE_INVERT);
  }

  private void setBraking() {
    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave.setIdleMode(IdleMode.kBrake);
  }

  private void setRamps() {
    double ramp = Util.getAndSetDouble("Drive Ramp", 0.25);
    leftMaster.setOpenLoopRampRate(ramp);
    leftSlave.setOpenLoopRampRate(ramp);
    rightMaster.setOpenLoopRampRate(ramp);
    rightSlave.setOpenLoopRampRate(ramp);
  }

  private void setFollowers() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
  }
}
