/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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
  private static CANSparkMax climber;

  public SubsystemClimb() {
    climber = new CANSparkMax(Constants.CLIMBER_ID, MotorType.kBrushless);
  }

  public double ascendByController(Joystick controller) {
    double speed = Xbox.RT(controller) - Xbox.LT(controller);
    climber.set(speed);
    return climber.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
