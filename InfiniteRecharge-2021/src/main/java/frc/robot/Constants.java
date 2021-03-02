/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Drivetrain Motor IDs
     */
    public static final int
        DRIVE_RIGHT_MASTER_ID = 3, //spark
        DRIVE_RIGHT_SLAVE_ID  = 4, //spark
        DRIVE_LEFT_MASTER_ID  = 1, //spark
        DRIVE_LEFT_SLAVE_ID   = 2; //spark

    /**
     * Turret Motor IDs
     */
    public static final int
        TURRET_YAW_ID      = 8,
        TURRET_PITCH_ID    = 9,
        TURRET_FLYWHEEL_ID = 7; //spark

    /**
     * Climber Motor IDs
     */
    public static final int
        CLIMBER_SCISSOR_ID = 5, //spark
        CLIMBER_WINCH_ID = 6;   //spark

    /**
     * Spinner Motor IDs
     */
    public static final int
        SPINNER_ID = 10;

  /**
   * Intake IDs
   */
    public static final int
        MAININTAKE_ID = 0,
        FEEDINTAKE_ID = 0;
    
    /**
     * Intake Motor IDs
     */
    public static final int
        EATER_ID = 11,
        SLAPPER_ID = 12;

    /**
     * Feeder Motor IDs
     */
    public static final int
        BEATER_ID = 13,
        FEEDER_ID = 14;

    /**
     * Jevois ID and baud rate
     */
    public static final int
        JEVOIS_BAUD_RATE = 115200;

    public static final Port
        JEVOIS_PORT = Port.kUSB;
    
    /**
     * Drivetrain motor inverts
     */
    public static final boolean
        DRIVE_RIGHT_MASTER_INVERT = false,
        DRIVE_RIGHT_SLAVE_INVERT  = false,
        DRIVE_LEFT_MASTER_INVERT  = true,
        DRIVE_LEFT_SLAVE_INVERT   = true;

    /**
     * Turret Motor Inverts
     */
    public static final boolean
        TURRET_YAW_INVERT      = false,
        TURRET_PITCH_INVERT    = true,
        TURRET_FLYWHEEL_INVERT = true;

    /**
     * Climber Inverts
     */
    public static final boolean
        CLIMBER_SCISSOR_INVERT = false,
        CLIMBER_WINCH_INVERT   = true;

    /**
     * Spinner Inverts
     */
    public static final boolean
        SPINNER_INVERT = false;

    /**
     * Intake Motor Inverts
     */
    public static final boolean
        EATER_INVERT   = false,
        SLAPPER_INVERT = false;

    /**
     * Feeder Motor Inverts
     */
    public static final boolean
        BEATER_INVERT = false,
        FEEDER_INVERT = true;

    /**
     * Braking Values
     */
    public static final boolean
        INTAKE_BRAKING = true,
        FEEDER_BRAKING = true;

    /**
     * Amp Limits
     */
    public static int
        FLYWHEEL_AMP_LIMIT = 50;
    
    /*
    * Climber Values
    */
    public static final double
        LOWEST_HEIGHT = 0,
        ON_WHEEL_HEIGHT = 30,
        ABOVE_WHEEL_HEIGHT = 32,
        HIGHEST_HEIGHT = 60;

    /**
     * Drivetrain amp limits
     */
    public static final int
        DRIVE_AMP_LIMIT = 60;

    /**
     * Turret Constant Inhibitors and Amp Limits
     */
    public static final double
        TURRET_YAW_ABSOLUTE_MAX_OUTPUT = 0.5;

    public static final int
        TURRET_YAW_AMP_LIMIT = 100;

    /*
    * FORMAT: Red_Min, Green_Min, Blue_Min, Red_Max, Green_Max, Blue_Max.
    */
    public static final int[]
        TARGET_RED    = { 99 , 90 , 35 , 119, 110, 55  },
        TARGET_GREEN  = { 42 , 126, 56 , 62 , 146, 76  },
        TARGET_BLUE   = { 36 , 103, 85 , 56 , 123, 105 },
        TARGET_YELLOW = { 75 , 127, 22 , 95 , 147, 42  };
    
    /**
     * Physical traits of the robot
     */
    public static final int
        ROBOT_WEIGHT_POUND_FORCE = 120,
        TURRET_YAW_DEGREES = 345,
        DEFAULT_TURRET_YAW_TICKS = 1615628,
        YAW_FACE_FORWARD_DEGREES = 170,
        DEFAULT_TURRET_PITCH_TICKS = 9000,
        TURRET_YAW_ALLOWABLE_ERROR = 75000,
        TURRET_PITCH_ALLOWABLE_ERROR = 350,
        SPARK_MINIMUM_VOLTAGE = 8, //used to check if motor controllers are connected
        TALON_MINIMUM_AMPERAGE = 1;

    public static final double 
        FLYWHEEL_GEAR_RATIO = 1.6071,
        DRIVETRAIN_ALLOWABLE_ERROR = 1,
        CLIMBER_WINCH_ALLOWABLE_ERROR = 0.5;

    /**
     * Auto values
     */
    public static final int
        DISTANCE_INIT_LINE_TO_ALLIANCE_WALL = 120,
        KIWILIGHT_STABLE_DEGREES = 1, //degrees
        KIWILIGHT_STABLE_TIME = 0,
        FLYWHEEL_STABLE_RPM = 5600,
        AUTO_OVERREV_EXTRA_RPM = 500,
        AUTO_INIT_YAW_TARGET = -624258,
        TURRET_TARGET_TICKS_PER_INCH = -1399, //ticks to turn per inch away from wall
        AUTO_INIT_PITCH_TARGET = -5000,
        AUTO_DEEP_TRENCH_DISTANCE = -150,
        AUTO_SHALLOW_TRENCH_DISTANCE = -156,
        TRENCH_AUTO_WAIT_TIME = 500,
        SHOOT_PAYLOAD_DEFAULT_WAIT_TIME = 15000; //15 seconds or 15,000 ms

    /**
     * Temporary auto
     */
    public static final int
        KIWILIGHT_SOFT_ALIGN_DEGREES = 12;

    /**
     * More auto values but doubles
     */
    public static final double 
        DRIVE_ROTATIONS_PER_INCH = 0.472,
        DRIVE_AUTO_INHIBITOR = 0.8,
        EMULATE_PATH_MAX_POINT_DISTANCE = 1;

    /**
     * More auto values but booleans
     */
    public static final boolean
        AUTO_OVERREV_TURRET = false; //if set to true, flywheel speed will be increased by 500 RPM and feeder will not stop to regain FW velocity.


    /**
     * Emulation Constants
     */
    public static final double
        PATH_RECORDER_DISTANCE_INTERVAL = 3,
        DRIVETRAIN_WHEEL_BASE_WIDTH = 20;

    public static final int
        EMULATE_POINT_SKIP_LIMIT = 5,
        EMULATE_MAX_HEADING_TO_TURN_DIFFERENCE = 75;

    public static final String
        EMULATE_DEFAULT_POINTS_FILE_PATH = "/home/lvuser/points.txt",
        EMULATE_RESULTS_FILE_PATH = "/home/lvuser/results.txt";

    /**
     * Judgement auto
     */
    public static final int
        JUDGEMENT_AUTO_BALLS_TO_SHOOT = 5,
        JUDGEMENT_AUTO_SHOOT_DRIVE_DISTANCE = 132, //inches
        JUDGEMENT_AUTO_YAW_TARGET = 175000, //ticks
        JUDGEMENT_AUTO_PITCH_TARGET = -8916; //ticks

    public static final double
        JUDGEMENT_AUTO_SHOOT_DRIVE_POWER = 0.15; //unit in/s

    public static final String
        JUDGEMENT_AUTO_DRIVE_TO_POWER_CELLS_PATH_FILE = "/home/lvuser/ja_driveToCells.txt",
        JUDGEMENT_AUTO_DRIVE_TO_SITE_PATH_FILE        = "/home/lvuser/ja_driveToSite.txt",
        JUDGEMENT_AUTO_DRIVE_BACK_TO_START_PATH_FILE  = "/home/lvuser/ja_driveBackToStart.txt";

    /**
     * Traditional Judgment Auto
     */
    public static final int
        TRAD_JUDGEMENT_AUTO_BALLS_TO_SHOOT = 5,
        TRAD_JUDGEMENT_AUTO_AVOID_DISTANCE = 36, //inches
        TRAD_JUDGEMENT_AUTO_YAW_TARGET = 712413, //ticks
        TRAD_JUDGEMENT_AUTO_PITCH_TARGET = -8916;

    public static final double
        TRAD_JUDGEMENT_AUTO_AVOID_POWER = 0.5;

    public static final String
        TRAD_JUDGEMENT_AUTO_DRIVE_TO_POWER_CELLS_FILE = "/home/lvuser/ja2_collectCells.txt",
        TRAD_JUDGEMENT_AUTO_DRIVE_TO_SITE_FILE        = "/home/lvuser/ja2_driveToSite.txt";

    /**
     * Camera Stuff
     */
    public static final boolean
        DRIVE_CAMERA_AUTOMATIC_EXPOSURE = false;
}
