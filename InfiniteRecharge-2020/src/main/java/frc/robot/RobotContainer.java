/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.Auto;
import frc.robot.commands.ButtonCommandGroupRunIntakeFeeder;
import frc.robot.commands.CyborgCommandAlignTurret;
import frc.robot.commands.CyborgCommandCalibrateTurretPitch;
import frc.robot.commands.CyborgCommandCalibrateTurretYaw;
import frc.robot.commands.CyborgCommandFlywheelVelocity;
import frc.robot.commands.CyborgCommandPositionControl;
import frc.robot.commands.CyborgCommandTestScissorPositition;
import frc.robot.commands.CyborgCommandZeroTurret;
import frc.robot.commands.SemiManualCommandRunWinch;
import frc.robot.commands.ToggleCommandDriveClimber;
import frc.robot.commands.ToggleCommandDriveFlywheel;
import frc.robot.subsystems.SubsystemClimb;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemFeeder;
import frc.robot.subsystems.SubsystemFlywheel;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemReceiver;
import frc.robot.subsystems.SubsystemSpinner;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Xbox;

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
   * Dashboard Items
   */
  private SendableChooser<Command> autoChooser;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //auto chooser bro
    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("The Bare Minimum", Auto.theBareMinimum(SUB_DRIVE, SUB_TURRET, SUB_FLYWHEEL, SUB_INTAKE, SUB_FEEDER, SUB_RECEIVER));
    autoChooser.addOption("Drive and Zero", Auto.autoInitCommand(SUB_DRIVE, SUB_TURRET));
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /**
     * Things that run automatically
     */
    CyborgCommandFlywheelVelocity driveFlywheelRPM = new CyborgCommandFlywheelVelocity(SUB_FLYWHEEL);
    SUB_FLYWHEEL.setDefaultCommand(driveFlywheelRPM);

    /**
     * DRIVER controls
     */
    //manual commands
    SUB_DRIVE.setDefaultCommand(
      new RunCommand(() -> SUB_DRIVE.DriveTankByController(DRIVER), SUB_DRIVE)
    );

    /**
     * OPERATOR controls
     */
    //manual commands
    SUB_TURRET.setDefaultCommand(
      new RunCommand(() -> SUB_TURRET.moveTurret(OPERATOR), SUB_TURRET)
    );

    SUB_INTAKE.setDefaultCommand(
      new ButtonCommandGroupRunIntakeFeeder(SUB_INTAKE, SUB_FEEDER, OPERATOR)
    );

    SemiManualCommandRunWinch semiManualWinchCommand = new SemiManualCommandRunWinch(SUB_CLIMB, DRIVER);
    SUB_CLIMB.setDefaultCommand(semiManualWinchCommand);

    //toggle commands
    JoystickButton toggleFlywheel = new JoystickButton(OPERATOR, Xbox.START);
      toggleFlywheel.toggleWhenPressed(new ToggleCommandDriveFlywheel(SUB_FLYWHEEL, 0));

    ToggleCommandDriveClimber climberManualDrive = new ToggleCommandDriveClimber(SUB_CLIMB, SUB_TURRET, OPERATOR);
    JoystickButton toggleClimberManual = new JoystickButton(OPERATOR, Xbox.BACK);
      toggleClimberManual.toggleWhenPressed(climberManualDrive);

    //button commands
    CyborgCommandAlignTurret alignTurret = new CyborgCommandAlignTurret(SUB_TURRET, SUB_RECEIVER);
    JoystickButton toggleAlign = new JoystickButton(OPERATOR, Xbox.RB);
      toggleAlign.toggleWhenPressed(alignTurret);

    /**
     * Dashboard Buttons
     */
    SmartDashboard.putData("Calibrate Turret Yaw", new CyborgCommandCalibrateTurretYaw(SUB_TURRET));
    SmartDashboard.putData("Calibrate Turret Pitch", new CyborgCommandCalibrateTurretPitch(SUB_TURRET));
    SmartDashboard.putData("Zero Scissor and Winch Encoders", new InstantCommand(() -> SUB_CLIMB.zeroEncoders(), SUB_CLIMB));
    SmartDashboard.putData("Test Scissor PID", new CyborgCommandTestScissorPositition(SUB_CLIMB, OPERATOR));
    SmartDashboard.putData("Zero Turret", new CyborgCommandZeroTurret(SUB_TURRET));

    SmartDashboard.putData("Toggle Winch", climberManualDrive);
    SmartDashboard.putData("Drive Flywheel RPM", driveFlywheelRPM);
    SmartDashboard.putData("Align Turret", alignTurret);
    SmartDashboard.putData("Run  Winch", semiManualWinchCommand);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
