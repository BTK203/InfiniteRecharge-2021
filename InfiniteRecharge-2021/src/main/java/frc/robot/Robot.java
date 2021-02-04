/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Util;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static RobotContainer robotContainer;

  public static RobotContainer getRobotContainer() {
    return robotContainer;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    DriverStation.reportWarning("ROBOT STARTED, GOOD LUCK", false);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.printAllSystemsGo();
    robotContainer.updateDriveSchemeIndicators();
    robotContainer.updatePositionIndicator();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    DriverStation.reportWarning("AUTO STARTING", false);
    DriverStation.reportWarning("AAAAAAAAAAAAA", false);
    robotContainer.startAuto();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    DriverStation.reportWarning("TELEOP STARTING", false);
    robotContainer.cancelAuto();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    boolean[] tests = new boolean[6];
    tests[0] = Util.assertEquals("18 -> 18", Util.getAcuteSuppliment(18), 18.0);
    tests[1] = Util.assertEquals("-91 -> -1", Util.getAcuteSuppliment(-91), -1.0);
    tests[2] = Util.assertEquals("456 -> 6", Util.getAcuteSuppliment(456), 6.0);

    boolean allTests = tests[0] && tests[1] && tests[2];
    DriverStation.reportError((allTests ? "ALL TESTS PASSED" : "SOME TESTS FAILED"), false);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
