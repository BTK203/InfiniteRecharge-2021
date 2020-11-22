/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;



import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Util;
import frc.robot.util.Xbox;

public class SubsystemDrive extends SubsystemBase {
  private static CANSparkMax leftMaster;
  private static CANSparkMax leftSlave;
  private static CANSparkMax rightMaster;
  private static CANSparkMax rightSlave;

  private AHRS navX;

  /**
   * Creates a new SubsystemDrive.
   */
  public SubsystemDrive() {
    leftMaster = new CANSparkMax(Constants.DRIVE_LEFT_MASTER_ID, MotorType.kBrushless);
    leftSlave = new CANSparkMax(Constants.DRIVE_LEFT_SLAVE_ID, MotorType.kBrushless);
    rightMaster = new CANSparkMax(Constants.DRIVE_RIGHT_MASTER_ID, MotorType.kBrushless);
    rightSlave = new CANSparkMax(Constants.DRIVE_RIGHT_SLAVE_ID, MotorType.kBrushless);

    navX = new AHRS(Port.kUSB);

    setBraking();
    setRamps();
    setFollowers();
    setAmpLimits();
    setInverts();
  }

  /**
   * Runs with every robot frame.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Right Position", rightMaster.getEncoder().getPosition());
    SmartDashboard.putNumber("Left Position", leftMaster.getEncoder().getPosition());

    SmartDashboard.putNumber("Right Output", rightMaster.getAppliedOutput());
    SmartDashboard.putNumber("Left Output", leftMaster.getAppliedOutput());

    SmartDashboard.putNumber("Right Amps", rightMaster.getOutputCurrent());
    SmartDashboard.putNumber("Left Amps", leftMaster.getOutputCurrent());

    SmartDashboard.putBoolean("NavX Connected", getNavXConnected());
  }

  /**
   * Prints dashboard indicators indicating whether the subsystem is ready for a match.
   * Indicators are to be used for pre-match only. They do not provide an accurite indication
   * of the state of a subsystem in mid match.
   * @return true if the system is ready for a match, false otherwise.
   */
  public boolean getSystemIsGo() {
    boolean leftMasterConnected = leftMaster.getBusVoltage() > Constants.SPARK_MINIMUM_VOLTAGE;
    boolean rightMasterConnected = rightMaster.getBusVoltage() > Constants.SPARK_MINIMUM_VOLTAGE;
    boolean leftSlaveConnected = leftSlave.getBusVoltage() > Constants.SPARK_MINIMUM_VOLTAGE;
    boolean rightSlaveConnected = rightSlave.getBusVoltage() > Constants.SPARK_MINIMUM_VOLTAGE;
    boolean navXConnected = navX.isConnected();

    SmartDashboard.putBoolean("Left Master Connected", leftMasterConnected);
    SmartDashboard.putBoolean("Right Master Connected", rightMasterConnected);
    SmartDashboard.putBoolean("Left Slave Connected", leftSlaveConnected);
    SmartDashboard.putBoolean("Right Slave Connected", rightSlaveConnected);

    return 
      leftMasterConnected &&
      rightMasterConnected &&
      leftSlaveConnected &&
      rightSlaveConnected && 
      navXConnected;
  }

  public boolean getNavXConnected() {
    return navX.isConnected();
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

    double inhibitor = Util.getAndSetDouble("Drive Inhibitor", 1);
    driveRight *= inhibitor;
    driveLeft *= inhibitor;

    leftMaster.set(driveLeft);
    leftSlave.set(driveLeft);
    rightMaster.set(driveRight);
    rightSlave.set(driveRight);
  }

  public void driveTankTrue(Joystick left, Joystick right) {
    setInverts();

    double rawLeftDrive = left.getY() * -1;
    double rawRightDrive = right.getY() * -1;

    //apply deadzone since the attack joysticks are sensitive
    double deadzone = Util.getAndSetDouble("Logitech Attack Deadzone", 0.025);
    if(Math.abs(rawLeftDrive) < deadzone) {
      rawLeftDrive = 0;
    }

    if(Math.abs(rawRightDrive) < deadzone) {
      rawRightDrive = 0;
    }

    //apply exponential sensitivity
    int sensitivity = (int) Util.getAndSetDouble("True Tank Sensitivity", 3);
    double leftDrive = Math.pow(rawLeftDrive, sensitivity);
    double rightDrive = Math.pow(rawRightDrive, sensitivity);

    //make sure that direction holds true (because even powers cannot be negative)
    if(sensitivity % 2 == 0) {
      if(rawLeftDrive < 0 && leftDrive > 0) { //left drive needs to be negated
        leftDrive *= -1;
      }

      if(rawRightDrive < 0 && rightDrive > 0) { //right drive needs to be negated
        rightDrive *= -1;
      }
    }

    double inhibitor = Util.getAndSetDouble("Drive Inhibitor", 1);
    leftDrive *= inhibitor;
    rightDrive *= inhibitor;

    leftMaster.set(leftDrive);
    leftSlave.set(leftDrive);
    rightMaster.set(rightDrive);
    rightSlave.set(rightDrive);
  }

  /**
   * Sets the output of the right drive motors.
   * @param output output (-1 to 1) to set the motors to.
   */
  public void setRightPercentOutput(double output) {
    rightMaster.set(output);
    rightSlave.set(output);
  }

  /**
   * Sets the output of the left drive motors.
   * @param output output (-1 to 1) to set the motors to.
   */
  public void setLeftPercentOutput(double output) {
    leftMaster.set(output);
    leftSlave.set(output);
  }

  /**
   * Sets the target position of the left motors.
   * @param leftPosition target position (rotations) to set the motors to.
   */
  public void setLeftPosition(double leftPosition) {
    leftMaster.getPIDController().setReference(leftPosition, ControlType.kPosition);
  }

  /**
   * Sets the target position of the right motors.
   * @param rightPosition target position (rotations) to set the motors to.
   */
  public void setRightPosition(double rightPosition) {
    rightMaster.getPIDController().setReference(rightPosition, ControlType.kPosition);
  }

  /**
   * Sets the encoder counts of the motors to 0.
   */
  public void zeroEncoders() {
    leftMaster.getEncoder().setPosition(0);
    rightMaster.getEncoder().setPosition(0);
  }

  /**
   * Returns the current position (rotations) of the left motors.
   */
  public double getLeftPosition() {
    return leftMaster.getEncoder().getPosition();
  }

  /**
   * Returns the current position (rotations) of the right motor.
   */
  public double getRightPosition() {
    return rightMaster.getEncoder().getPosition();
  }

  /**
   * Sets the PID constants of all motors.
   * @param kP new P gain
   * @param kI new I gain
   * @param kD new D gain
   * @param kF new F gain
   * @param iZone Proximity to target at which I takes effect
   * @param outLimit maximum percent output of the motors.
   */
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

  public double getGyroAngle() {
    return navX.getAngle();
  }

  /**
   * Sets the inverts of the drive motors.
   */
  private void setInverts() {
    leftMaster.setInverted(Constants.DRIVE_LEFT_MASTER_INVERT);
    leftSlave.setInverted(Constants.DRIVE_LEFT_SLAVE_INVERT);
    rightMaster.setInverted(Constants.DRIVE_RIGHT_MASTER_INVERT);
    rightSlave.setInverted(Constants.DRIVE_RIGHT_SLAVE_INVERT);
  }

  /**
   * Sets all motors to braking mode.
   */
  private void setBraking() {
    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Configures the ramp rate of the motors.
   */
  private void setRamps() {
    double ramp = Util.getAndSetDouble("Drive Ramp", 0.25);
    leftMaster.setOpenLoopRampRate(ramp);
    leftSlave.setOpenLoopRampRate(ramp);
    rightMaster.setOpenLoopRampRate(ramp);
    rightSlave.setOpenLoopRampRate(ramp);
  }

  /**
   * Sets the amp limits of the motors.
   */
  private void setAmpLimits() {    
    leftMaster.setSmartCurrentLimit(Constants.DRIVE_AMP_LIMIT);
    leftSlave.setSmartCurrentLimit(Constants.DRIVE_AMP_LIMIT);
    rightMaster.setSmartCurrentLimit(Constants.DRIVE_AMP_LIMIT);
    rightSlave.setSmartCurrentLimit(Constants.DRIVE_AMP_LIMIT);
  }

  /**
   * Marks the slave motors as "following" the master motors.
   */
  private void setFollowers() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
  }

  /**
   * METHODS FOR DEVELOPMENT PURPOSES ONLY
   */

  public void driveJustMasters(Joystick controller) {
    double drive = Xbox.RT(controller) - Xbox.LT(controller);
    leftMaster.set(drive);
    rightMaster.set(drive);
  }

  public void driveJustSlaves(Joystick controller) {
    double drive = Xbox.RT(controller) - Xbox.LT(controller);
    leftSlave.set(drive);
    rightSlave.set(drive);
  }
}
