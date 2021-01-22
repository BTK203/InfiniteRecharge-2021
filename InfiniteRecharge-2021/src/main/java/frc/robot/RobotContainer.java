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
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.auto.BareMinimumAuto;
import frc.robot.auto.IAuto;
import frc.robot.auto.InitAuto;
import frc.robot.auto.SixBallSimpleAuto;
import frc.robot.auto.TrenchAuto;
import frc.robot.commands.ButtonCommandGroupRunIntakeFeeder;
import frc.robot.commands.ButtonCommandMoveClimber;
import frc.robot.commands.CyborgCommandAlignTurret;
import frc.robot.commands.CyborgCommandCalibrateTurretPitch;
import frc.robot.commands.CyborgCommandCalibrateTurretYaw;
import frc.robot.commands.CyborgCommandDriveDistance;
import frc.robot.commands.CyborgCommandEmulatePath;
import frc.robot.commands.CyborgCommandFlywheelVelocity;
import frc.robot.commands.CyborgCommandRecordPath;
import frc.robot.commands.CyborgCommandSetTurretPosition;
import frc.robot.commands.CyborgCommandShootPayload;
import frc.robot.commands.CyborgCommandSmartDriveDistance;
import frc.robot.commands.CyborgCommandSmartMoveTurret;
import frc.robot.commands.CyborgCommandTestScissorPositition;
import frc.robot.commands.CyborgCommandTestVelocity;
import frc.robot.commands.CyborgCommandZeroTurret;
import frc.robot.commands.ManualCommandDrive;
import frc.robot.commands.ToggleCommandDriveClimber;
import frc.robot.enumeration.AutoMode;
import frc.robot.enumeration.DriveScheme;
import frc.robot.subsystems.SubsystemClimb;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemFeeder;
import frc.robot.subsystems.SubsystemFlywheel;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemReceiver;
import frc.robot.subsystems.SubsystemSpinner;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;
import frc.robot.util.Xbox;
import frc.robot.util.PositionTracker;
import frc.robot.util.Point2D;

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
  private final SubsystemTurret    SUB_TURRET   = new SubsystemTurret();
  private final SubsystemIntake    SUB_INTAKE   = new SubsystemIntake();
  private final SubsystemFeeder    SUB_FEEDER   = new SubsystemFeeder();
  private final SubsystemFlywheel  SUB_FLYWHEEL = new SubsystemFlywheel();
  private final SubsystemSpinner   SUB_SPINNER  = new SubsystemSpinner();
  private final SubsystemClimb     SUB_CLIMB    = new SubsystemClimb();
  private final SubsystemReceiver  SUB_RECEIVER = new SubsystemReceiver();
  // private final CameraHub          CAMERA_HUB   = new CameraHub();

  /**
   * Utilities
   */
  private final PositionTracker POSITION_TRACKER = new PositionTracker(SUB_DRIVE);

  /**
   * Controllers
   */
  private final Joystick
    DRIVER   = new Joystick(0),
    OPERATOR = new Joystick(1),
    DRIVER2  = new Joystick(2);

  /**
   * Important Commands
   */
  private final CyborgCommandFlywheelVelocity driveFlywheelRPM = new CyborgCommandFlywheelVelocity(SUB_FLYWHEEL);

  /**
   * Dashboard Items
   */
  private SendableChooser<AutoMode> autoChooser;
  private SendableChooser<DriveScheme> driveChooser;

  /**
   * Auto
   */
  private IAuto currentAuto;
  private Command autoCommand;

  /**
   * Misc.
   */
  private boolean controllersGood;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureChoosers();

    currentAuto = new InitAuto(SUB_DRIVE, SUB_TURRET);
    autoCommand = currentAuto.getCommand();
    controllersGood = false;
  }

  public Point2D getRobotPositionAndHeading() {
    return POSITION_TRACKER.getPositionAndHeading();
  }

  /**
   * Schedules the autonomous command.
   */
  public void startAuto() {
    AutoMode desiredAuto = autoChooser.getSelected();

    //create the auto based on the enum
    switch(desiredAuto) {
      case INIT_ONLY:
        currentAuto = new InitAuto(SUB_DRIVE, SUB_TURRET);
        break;
      case THE_BARE_MINIMUM:
        currentAuto = new BareMinimumAuto(SUB_DRIVE, SUB_TURRET, SUB_RECEIVER, SUB_INTAKE, SUB_FEEDER, SUB_FLYWHEEL);
        break;
      case SIX_BALL_SIMPLE:
        currentAuto = new SixBallSimpleAuto(SUB_DRIVE, SUB_TURRET, SUB_RECEIVER, SUB_INTAKE, SUB_FEEDER, SUB_FLYWHEEL);
        break;
      case EIGHT_BALL_TRENCH:
        currentAuto = new TrenchAuto(SUB_DRIVE, SUB_TURRET, SUB_RECEIVER, SUB_INTAKE, SUB_FEEDER, SUB_FLYWHEEL);
        break;
      default:
        currentAuto = new InitAuto(SUB_DRIVE, SUB_TURRET);
        break;
    }

    autoCommand = currentAuto.getCommand();
    autoCommand.schedule();

    //start flywheel if necessary
    if(currentAuto.requiresFlywheel()) {
      if(Constants.AUTO_OVERREV_TURRET) {
        driveFlywheelRPM.overrideRPM(Util.getAndSetDouble("FW Velocity Target", 6000) + (double) Constants.AUTO_OVERREV_EXTRA_RPM);
      }

      driveFlywheelRPM.schedule();
    }
  }
  
  /**
   * Cancels the autonomous command.
   */
  public void cancelAuto() {
    if(currentAuto != null) {
      if(autoCommand.isScheduled()) {
        autoCommand.cancel();
      }
    } else {
      DriverStation.reportError("NO AUTO STARTED, THEREFORE NONE CANCELED.", false);
    }
  }

  private void zeroAllDrivetrain() {
    SUB_DRIVE.zeroEncoders();
    SUB_DRIVE.zeroGyro();
    POSITION_TRACKER.setPositionAndHeading(0, 0, 0);
  }

  /**
   * Returns the first driver joystick. May be an Xbox controller, but also
   * could be Logitech attack 3. 
   * @return 1st (right) drive joystick
   */
  public Joystick getDriver() {
    return DRIVER;
  }

  /**
   * Returns the operator joystick. Could be Xbox, might be Logitech atk3.
   * @return Operator Joystick.
   */
  public Joystick getOperator() {
    return OPERATOR;
  }

  /**
   * Returns the second Driver joystick, only used for true tank drive.
   * Will NOT always be defined as sometimes it will not be plugged in.
   * @return 2nd (left) drive joystick
   */
  public Joystick getDriver2() {
    return DRIVER2;
  }

  public DriveScheme getDriveScheme() {
    return driveChooser.getSelected();
  }

  public void updateDriveSchemeIndicators() {
    DriverStation ds = DriverStation.getInstance();

    //report names of devices
    SmartDashboard.putString("Device 1", ds.getJoystickName(0));
    SmartDashboard.putString("Device 2", ds.getJoystickName(1));
    SmartDashboard.putString("Device 3", ds.getJoystickName(2));

    switch(getDriveScheme()) {
      case RL: {
          SmartDashboard.putString("Drive Scheme Layout",
            "Device 1: Driver (xbox)  |  Device 2: Operator (xbox)"
          );

          //check to see if actual controller layout is good
          boolean controllersExist = Util.controllerExists(0);
          controllersGood = ds.getJoystickIsXbox(0) && controllersExist;
        }
        break;
      case TRUE_TANK: {
          SmartDashboard.putString("Drive Scheme Layout",
            "Device 1: Driver Right (Logitech Attack)  |  Device 2: Operator (xbox)  |  Device 3: Driver Left (Logitech Attack)"
          );

          //check to see if layout is good
          boolean controllersExist = Util.controllerExists(0) && Util.controllerExists(1) && Util.controllerExists(2);
          controllersGood = !ds.getJoystickIsXbox(0) && ds.getJoystickIsXbox(1) && !ds.getJoystickIsXbox(2) && controllersExist;
        }
        break;
    }

    String controllerWarning = (
      controllersGood?
      "Safe to enable." :
      "ENABLE AT YOUR OWN RISK"
    );
    SmartDashboard.putString("Controller Warning", controllerWarning);
    SmartDashboard.putBoolean("Controllers", controllersGood);
  }

  public void updatePositionIndicator() {
    SmartDashboard.putString("Robot Position", getRobotPositionAndHeading().toString());
  }

  /**
   * Returns true if the controller configuration is correct, false otherwise
   */
  public boolean controllersGood() {
    return controllersGood;
  }

  /**
   * Prints dashboard indicators indicating whether the robot subsystems are ready for a match.
   * Indicators are to be used for pre-match only. They do not provide an accurite indication
   * of the state of a subsystem in mid match.
   */
  public void printAllSystemsGo() {
    boolean 
      climbIsGo     = SUB_CLIMB.getSystemIsGo(),
      driveIsGo     = SUB_DRIVE.getSystemIsGo(),
      feederIsGo    = SUB_FEEDER.getSystemIsGo(),
      flywheelIsGo  = SUB_FLYWHEEL.getSystemIsGo(),
      intakeIsGo    = SUB_INTAKE.getSystemIsGo(),
      kiwilightIsGo = SUB_RECEIVER.getSystemIsGo(),
      spinnerIsGo   = SUB_SPINNER.getSystemIsGo(),
      turretIsGo    = SUB_TURRET.getSystemIsGo();

    boolean allSystemsGo = 
      climbIsGo &&
      driveIsGo &&
      feederIsGo &&
      flywheelIsGo &&
      intakeIsGo &&
      kiwilightIsGo &&
      spinnerIsGo &&
      turretIsGo &&
      controllersGood;
    
    SmartDashboard.putBoolean("All Systems Go", allSystemsGo);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /**
     * DRIVER controls
     */
    //manual commands
    SUB_DRIVE.setDefaultCommand(new ManualCommandDrive(SUB_DRIVE));



    SUB_SPINNER.setDefaultCommand(
      new RunCommand(() -> SUB_SPINNER.driveByController(DRIVER), SUB_SPINNER)
    );

    //button commands
    JoystickButton moveClimberUp = new JoystickButton(DRIVER, Xbox.START);
    moveClimberUp.toggleWhenPressed(new ButtonCommandMoveClimber(SUB_CLIMB, 1));

    JoystickButton moveClimberDown = new JoystickButton(DRIVER, Xbox.BACK);
    moveClimberDown.toggleWhenPressed(new ButtonCommandMoveClimber(SUB_CLIMB, -1));

    /**
     * OPERATOR controls
     */
    SUB_TURRET.setDefaultCommand(new CyborgCommandSmartMoveTurret(SUB_TURRET));

    SUB_INTAKE.setDefaultCommand(
      new ButtonCommandGroupRunIntakeFeeder(SUB_INTAKE, SUB_FEEDER, SUB_TURRET, OPERATOR)
    );

    //toggle commands
    JoystickButton toggleFlywheel = new JoystickButton(OPERATOR, Xbox.START);
      toggleFlywheel.toggleWhenPressed(driveFlywheelRPM);

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
    SmartDashboard.putData("Drive Flywheel RPM", driveFlywheelRPM);
    SmartDashboard.putData("Align Turret", new CyborgCommandAlignTurret(SUB_TURRET, SUB_RECEIVER, false));
    SmartDashboard.putData("Calibrate Turret Yaw", new CyborgCommandCalibrateTurretYaw(SUB_TURRET));
    SmartDashboard.putData("Calibrate Turret Pitch", new CyborgCommandCalibrateTurretPitch(SUB_TURRET));
    SmartDashboard.putData("Zero Scissor and Winch Encoders", new InstantCommand(() -> SUB_CLIMB.zeroEncoders(), SUB_CLIMB));
    SmartDashboard.putData("Test Scissor PID", new CyborgCommandTestScissorPositition(SUB_CLIMB, OPERATOR));
    SmartDashboard.putData("Zero Turret", new CyborgCommandZeroTurret(SUB_TURRET));
    SmartDashboard.putData("Set Turret Position", new CyborgCommandSetTurretPosition(SUB_TURRET, 500000, 0));
    SmartDashboard.putData("Drive Distance", new CyborgCommandDriveDistance(SUB_DRIVE, -132, 0.75));
    SmartDashboard.putData("Drive Distance Smart", new CyborgCommandSmartDriveDistance(SUB_DRIVE, 156, 1));
    SmartDashboard.putData("Test Drive Velocity", new CyborgCommandTestVelocity(SUB_DRIVE, 156));
    SmartDashboard.putData("Reset Fastest Speed", new InstantCommand(() -> SUB_DRIVE.resetFastestSpeed()));
    SmartDashboard.putData("Zero Yaw", new InstantCommand(() -> SUB_TURRET.setCurrentYawEncoderPosition(0), SUB_TURRET));
    SmartDashboard.putData("Zero Drivetrain Encoders", new InstantCommand(() -> SUB_DRIVE.zeroEncoders()));
    SmartDashboard.putData("Zero All Drivetrain", new InstantCommand(() -> zeroAllDrivetrain()));
    SmartDashboard.putData("Drive Just Masters", new RunCommand(() -> SUB_DRIVE.driveJustMasters(DRIVER), SUB_DRIVE));
    SmartDashboard.putData("Drive Just Slaves", new RunCommand(() -> SUB_DRIVE.driveJustSlaves(DRIVER), SUB_DRIVE));
    SmartDashboard.putData("Drive Straight", new CyborgCommandSmartDriveDistance(SUB_DRIVE, 60, 0.6));
    SmartDashboard.putData("Shoot Payload", new CyborgCommandShootPayload(SUB_INTAKE, SUB_FEEDER, SUB_FLYWHEEL, SUB_RECEIVER, SUB_TURRET, 3, 15000, false));
    SmartDashboard.putData("Toggle Winch", climberManualDrive);
    SmartDashboard.putData("Drive Flywheel RPM", driveFlywheelRPM);
    SmartDashboard.putData("Record Path", new CyborgCommandRecordPath(POSITION_TRACKER));
    SmartDashboard.putData("Emulate Path", new CyborgCommandEmulatePath(SUB_DRIVE));
  }
  
  private void configureChoosers() {
    //declare the different autos we will choose from
    autoChooser = new SendableChooser<AutoMode>();
    autoChooser.setDefaultOption("Bare Minimum Auto", AutoMode.THE_BARE_MINIMUM);
    autoChooser.addOption("Init Only", AutoMode.INIT_ONLY);
    autoChooser.addOption("Simple Six Ball", AutoMode.SIX_BALL_SIMPLE);
    autoChooser.addOption("Eight Ball", AutoMode.EIGHT_BALL_TRENCH);
    SmartDashboard.putData("Auto Mode", autoChooser);

    //declare the different drive schemes available
    driveChooser = new SendableChooser<DriveScheme>();
    driveChooser.setDefaultOption("Rocket League", DriveScheme.RL);
    driveChooser.addOption("True Tank", DriveScheme.TRUE_TANK);
    SmartDashboard.putData("Drive Scheme", driveChooser);

    //set drivetrain lock override to false for safety
    Preferences.getInstance().putBoolean("Override Drive Lock", false);
  }
}
