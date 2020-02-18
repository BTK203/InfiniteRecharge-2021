/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.ButtonCommandDriveSpinner;
import frc.robot.commands.ButtonCommandEat;
import frc.robot.commands.ButtonCommandFeed;
import frc.robot.commands.ButtonCommandSpit;
import frc.robot.commands.CyborgCommandFlywheelVelocity;
import frc.robot.commands.CyborgCommandPositionControl;
import frc.robot.commands.ToggleCommandDriveFlywheel;
import frc.robot.commands.ToggleCommandRunWinch;
import frc.robot.subsystems.SubsystemClimb;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemFeeder;
import frc.robot.subsystems.SubsystemFlywheel;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemReceiver;
import frc.robot.subsystems.SubsystemSpinner;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Xbox;
import frc.robot.CameraHub;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /**
   * Subsystems
   */
  private final SubsystemDrive     SUB_DRIVE    = new SubsystemDrive();
  private final SubsystemIntake    SUB_INTAKE   = new SubsystemIntake();
  private final SubsystemFeeder    SUB_FEEDER   = new SubsystemFeeder();
  private final SubsystemTurret    SUB_TURRET   = new SubsystemTurret();
  private final SubsystemFlywheel  SUB_FLYWHEEL = new SubsystemFlywheel();
  private final SubsystemSpinner   SUB_SPINNER  = new SubsystemSpinner();
  private final SubsystemClimb     SUB_CLIMB    = new SubsystemClimb();
  private final SubsystemReceiver  SUB_RECEIVER = new SubsystemReceiver();
  // private final CameraHub          CAMERA_HUB   = new CameraHub();

  /**
   * Controllers
   */
  private final Joystick
    DRIVER   = new Joystick(0),
    OPERATOR = new Joystick(1);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /**
     * Manual Commands
     */
    SUB_DRIVE.setDefaultCommand(
      new RunCommand(() -> SUB_DRIVE.DriveTankByController(DRIVER), SUB_DRIVE)
    );

    SUB_CLIMB.setDefaultCommand(
      new RunCommand(() -> SUB_CLIMB.ascendByController(DRIVER), SUB_CLIMB)
    );

    SUB_TURRET.setDefaultCommand(
      new RunCommand(() -> SUB_TURRET.moveTurret(OPERATOR), SUB_TURRET)
    );

    /**
     * Button Commands
     */
    // JoystickButton toggleManualTurretControl = new JoystickButton(OPERATOR, Xbox.RSTICK);
    //   toggleManualTurretControl.toggleWhenPressed(
    //     new RunCommand(() -> SUB_TURRET.moveTurret(OPERATOR), SUB_TURRET)
    //   );

     ToggleCommandRunWinch winchCommand = new ToggleCommandRunWinch(SUB_CLIMB, DRIVER);
     JoystickButton toggleWinching = new JoystickButton(DRIVER, Xbox.BACK);
      toggleWinching.toggleWhenPressed(winchCommand);

    CyborgCommandFlywheelVelocity driveFlywheelRPM = new CyborgCommandFlywheelVelocity(SUB_FLYWHEEL);
    JoystickButton toggleFlywheel = new JoystickButton(OPERATOR, Xbox.START);
      toggleFlywheel.toggleWhenPressed(driveFlywheelRPM);

    JoystickButton feederEat = new JoystickButton(OPERATOR, Xbox.A);
      feederEat.whileHeld(new ButtonCommandEat(SUB_INTAKE, SUB_FEEDER, OPERATOR));

    JoystickButton feederFeed = new JoystickButton(OPERATOR, Xbox.X);
      feederFeed.whileHeld(new ButtonCommandFeed(SUB_INTAKE, SUB_FEEDER, OPERATOR));

    JoystickButton feederSpit = new JoystickButton(OPERATOR, Xbox.B);
      feederSpit.whileHeld(new ButtonCommandSpit(SUB_INTAKE, SUB_FEEDER));

    JoystickButton spinnerSpinLeft = new JoystickButton(OPERATOR, Xbox.LB);
      spinnerSpinLeft.toggleWhenPressed(new CyborgCommandPositionControl(SUB_SPINNER));

    // JoystickButton spinnerSpinRight = new JoystickButton(OPERATOR, Xbox.RB);
    //   spinnerSpinRight.whileHeld(new ButtonCommandDriveSpinner(SUB_SPINNER, true));

    /**
     * Dashboard Buttons
     */
   // SmartDashboard.putData("Toggle Winch", winchCommand);
    SmartDashboard.putData("Drive Flywheel RPM", driveFlywheelRPM);
    SmartDashboard.putData("Drive Flywheel PO", new ToggleCommandDriveFlywheel(SUB_FLYWHEEL));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
