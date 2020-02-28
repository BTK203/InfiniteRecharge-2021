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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.ButtonCommandDriveSpinner;
import frc.robot.commands.ButtonCommandEat;
import frc.robot.commands.ButtonCommandFeed;
import frc.robot.commands.ButtonCommandGroupRunIntakeFeeder;
import frc.robot.commands.ButtonCommandSpit;
import frc.robot.commands.CyborgCommandAlignTurret;
import frc.robot.commands.CyborgCommandCalibrateTurretPitch;
import frc.robot.commands.CyborgCommandCalibrateTurretYaw;
import frc.robot.commands.CyborgCommandFlywheelVelocity;
import frc.robot.commands.CyborgCommandPositionControl;
import frc.robot.commands.CyborgCommandTestScissorPositition;
import frc.robot.commands.SemiManualCommandRunWinch;
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

    ToggleCommandRunWinch winchCommand = new ToggleCommandRunWinch(SUB_CLIMB, OPERATOR);
    SUB_CLIMB.setDefaultCommand(winchCommand);

    SUB_TURRET.setDefaultCommand(
      new RunCommand(() -> SUB_TURRET.moveTurret(OPERATOR), SUB_TURRET)
    );

    SUB_INTAKE.setDefaultCommand(
      new ButtonCommandGroupRunIntakeFeeder(SUB_INTAKE, SUB_FEEDER, OPERATOR)
    );

    /**
     * Button Commands
     */
    // JoystickButton toggleManualTurretControl = new JoystickButton(OPERATOR, Xbox.RSTICK);
    //   toggleManualTurretControl.toggleWhenPressed(
    //     new RunCommand(() -> SUB_TURRET.moveTurret(OPERATOR), SUB_TURRET)
    //   );

    SemiManualCommandRunWinch semiManualWinchCommand = new SemiManualCommandRunWinch(SUB_CLIMB, OPERATOR);
     JoystickButton toggleClimberSemiManual = new JoystickButton(OPERATOR, Xbox.BACK);
      toggleClimberSemiManual.toggleWhenPressed(semiManualWinchCommand);

    CyborgCommandFlywheelVelocity driveFlywheelRPM = new CyborgCommandFlywheelVelocity(SUB_FLYWHEEL);
    JoystickButton toggleFlywheel = new JoystickButton(OPERATOR, Xbox.START);
      toggleFlywheel.toggleWhenPressed(driveFlywheelRPM);

    JoystickButton toggleAlign = new JoystickButton(OPERATOR, Xbox.RB);
      toggleAlign.toggleWhenPressed(new CyborgCommandAlignTurret(SUB_TURRET, SUB_RECEIVER, DRIVER, OPERATOR));

    JoystickButton spinnerSpinLeft = new JoystickButton(OPERATOR, Xbox.LB);
      spinnerSpinLeft.toggleWhenPressed(new CyborgCommandPositionControl(SUB_SPINNER));

    // JoystickButton spinnerSpinRight = new JoystickButton(OPERATOR, Xbox.RB);
    //   spinnerSpinRight.whileHeld(new ButtonCommandDriveSpinner(SUB_SPINNER, true));

    /**
     * Dashboard Buttons
     */
    SmartDashboard.putData("Toggle Winch", winchCommand);
    SmartDashboard.putData("Drive Flywheel RPM", driveFlywheelRPM);
    SmartDashboard.putData("Drive Flywheel PO", new ToggleCommandDriveFlywheel(SUB_FLYWHEEL));
    SmartDashboard.putData("Align Turret", new CyborgCommandAlignTurret(SUB_TURRET, SUB_RECEIVER, DRIVER, OPERATOR));
    SmartDashboard.putData("Calibrate Turret Yaw", new CyborgCommandCalibrateTurretYaw(SUB_TURRET));
    SmartDashboard.putData("Calibrate Turret Pitch", new CyborgCommandCalibrateTurretPitch(SUB_TURRET));
    SmartDashboard.putData("Test Scissors PID", new CyborgCommandTestScissorPositition(SUB_CLIMB, OPERATOR));
    SmartDashboard.putData("Zero Scissor and Winch Encoders", new InstantCommand(() -> SUB_CLIMB.zeroScissorEncoders(), SUB_CLIMB));
    SmartDashboard.putData("Run  Winch", semiManualWinchCommand);
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
