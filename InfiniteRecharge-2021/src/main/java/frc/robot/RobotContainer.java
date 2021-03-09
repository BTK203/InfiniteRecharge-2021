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
import frc.robot.auto.JudgementAuto;
import frc.robot.auto.SixBallSimpleAuto;
import frc.robot.auto.TraditionalJudgementAuto;
import frc.robot.auto.TrenchAuto;
import frc.robot.commands.ButtonCommandGroupRunIntakeFeeder;
import frc.robot.commands.ButtonCommandMoveClimber;
import frc.robot.commands.ConstantCommandDriveIntake;
import frc.robot.commands.CyborgCommandAlignTurret;
import frc.robot.commands.CyborgCommandCalibrateTurretPitch;
import frc.robot.commands.CyborgCommandCalibrateTurretYaw;
import frc.robot.commands.CyborgCommandChaseBall;
import frc.robot.commands.CyborgCommandDriveDistance;
import frc.robot.commands.CyborgCommandEmulatePath;
import frc.robot.commands.CyborgCommandFlywheelVelocity;
import frc.robot.commands.CyborgCommandRecordPath;
import frc.robot.commands.CyborgCommandSetTurretPosition;
import frc.robot.commands.CyborgCommandSmartDriveDistance;
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
import frc.robot.subsystems.SubsystemJevois;
import frc.robot.subsystems.SubsystemSpinner;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.util.Util;
import frc.robot.util.Xbox;
import frc.robot.util.PositionTracker;
import frc.robot.util.Point2D;
import frc.robot.util.PVHost;

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
  private final SubsystemJevois    SUB_JEVOIS   = new SubsystemJevois();
  // private final CameraHub          CAMERA_HUB   = new CameraHub();

  /**
   * Utilities
   */
  private final PositionTracker POSITION_TRACKER     = new PositionTracker(SUB_DRIVE);
  private final PVHost          PATH_VISUALIZER_HOST = new PVHost(Constants.PV_PORT);

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

  /**
   * Updates the robot.
   * Updates the PathVisualizer client,
   * Updates the robot position,
   * Prints All Systems Go indicators,
   * Updates the drive scheme safety indicators
   * Updates the robot position indicator,
   */
  public void update() {
    POSITION_TRACKER.update();
    PATH_VISUALIZER_HOST.update(getRobotPositionAndHeading());
    printAllSystemsGo();
    updateDriveSchemeIndicators();
    updatePositionIndicator();
  }

  /**
   * Returns the robot's current position and heading. Units are in inches for the XY coordinates, and degrees for the heading.
   */
  public Point2D getRobotPositionAndHeading() {
    return POSITION_TRACKER.getPositionAndHeading();
  }

  /**
   * Returns the robot's PathVisualizer host.
   */
  public PVHost getPVHost() {
    return PATH_VISUALIZER_HOST;
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
      case FLEX_TIME:
        currentAuto = new JudgementAuto(SUB_DRIVE, SUB_INTAKE, SUB_FEEDER, SUB_TURRET, SUB_FLYWHEEL, SUB_RECEIVER);
        break;
      case FLEX_TIME_TRADIATIONAL:
        currentAuto = new TraditionalJudgementAuto(SUB_DRIVE, SUB_TURRET, SUB_INTAKE, SUB_FEEDER, SUB_FLYWHEEL, SUB_RECEIVER);
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

  public void zeroAllDrivetrain() {
    SUB_DRIVE.zeroEncoders();
    SUB_DRIVE.zeroGyro();
    POSITION_TRACKER.zeroPositionAndHeading();
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

  /**
   * Returns the current user-selected drive scheme. Will either be 
   * Rocket League drive(xbox controller), or True Tank drive(logitech attack joysticks)
   */
  public DriveScheme getDriveScheme() {
    return driveChooser.getSelected();
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
  private void printAllSystemsGo() {
    boolean 
      driveIsGo     = SUB_DRIVE.getSystemIsGo(),
      feederIsGo    = SUB_FEEDER.getSystemIsGo(),
      flywheelIsGo  = SUB_FLYWHEEL.getSystemIsGo(),
      intakeIsGo    = SUB_INTAKE.getSystemIsGo(),
      kiwilightIsGo = SUB_RECEIVER.getSystemIsGo(),
      spinnerIsGo   = SUB_SPINNER.getSystemIsGo(),
      turretIsGo    = SUB_TURRET.getSystemIsGo();

    boolean allSystemsGo = 
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
   * Updates the indicators for drive scheme (Controller layout, safe to enable, etc) on the dashboard.
   * This method is important because it might not be programmings fault if the robot drives full forward on enable and kills people
   */
  private void updateDriveSchemeIndicators() {
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

  /**
   * Updates the robot's position on the dashboard.
   */
  private void updatePositionIndicator() {
    SmartDashboard.putString("Robot Position", getRobotPositionAndHeading().toString());
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
    SUB_TURRET.setDefaultCommand(new RunCommand(() -> { SUB_TURRET.moveTurret(OPERATOR); }, SUB_TURRET));

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
    SmartDashboard.putData("Calibrate Turret Yaw", new CyborgCommandCalibrateTurretYaw(SUB_TURRET));
    SmartDashboard.putData("Calibrate Turret Pitch", new CyborgCommandCalibrateTurretPitch(SUB_TURRET));
    SmartDashboard.putData("Zero Turret", new CyborgCommandZeroTurret(SUB_TURRET));
    SmartDashboard.putData("Zero Yaw Encoder", new InstantCommand(() -> SUB_TURRET.setCurrentYawEncoderPosition(0), SUB_TURRET));
    SmartDashboard.putData("Zero Drivetrain Encoders", new InstantCommand(() -> SUB_DRIVE.zeroEncoders()));
    SmartDashboard.putData("Zero All Drivetrain", new InstantCommand(() -> zeroAllDrivetrain()));
    SmartDashboard.putData("Record Path", new CyborgCommandRecordPath(POSITION_TRACKER));
    SmartDashboard.putData("Emulate Path", new CyborgCommandEmulatePath(SUB_DRIVE));

    /**
     * Temporary dashboard buttons
     */
    SmartDashboard.putData("Test Turret Position", new CyborgCommandSetTurretPosition(SUB_TURRET, Constants.JUDGEMENT_AUTO_YAW_TARGET, Constants.JUDGEMENT_AUTO_PITCH_TARGET, true, SUB_RECEIVER));
    SmartDashboard.putData("Run Intake", new ConstantCommandDriveIntake(SUB_INTAKE, SUB_FEEDER));
    SmartDashboard.putData("Test DD", new CyborgCommandDriveDistance(SUB_DRIVE, Constants.JUDGEMENT_AUTO_SHOOT_DRIVE_DISTANCE, Constants.JUDGEMENT_AUTO_SHOOT_DRIVE_POWER));
    SmartDashboard.putData("Test SDD", new CyborgCommandSmartDriveDistance(SUB_DRIVE));
    SmartDashboard.putData("Test Chase", new CyborgCommandChaseBall(SUB_DRIVE, SUB_JEVOIS));
  }
  
  /**
   * Defines auto dropdown and drive scheme dropdown on the dashboard.
   */
  private void configureChoosers() {
    //declare the different autos we will choose from
    autoChooser = new SendableChooser<AutoMode>();
    autoChooser.setDefaultOption("Bare Minimum Auto", AutoMode.THE_BARE_MINIMUM);
    autoChooser.addOption("Init Only", AutoMode.INIT_ONLY);
    autoChooser.addOption("Simple Six Ball", AutoMode.SIX_BALL_SIMPLE);
    autoChooser.addOption("Eight Ball", AutoMode.EIGHT_BALL_TRENCH);
    autoChooser.addOption("Judgement Auto", AutoMode.FLEX_TIME); 
    autoChooser.addOption("Traditional Judgement Auto", AutoMode.FLEX_TIME_TRADIATIONAL);
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
