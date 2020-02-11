/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Drivetrain Motor IDs
     */
    public static final int
        DRIVE_RIGHT_MASTER_ID = 0,
        DRIVE_RIGHT_SLAVE_ID  = 0,
        DRIVE_LEFT_MASTER_ID  = 0,
        DRIVE_LEFT_SLAVE_ID   = 0;

    /**
     * Turret Motor IDs
     */
    public static final int
        TURRET_YAW_ID      = 0,
        TURRET_PITCH_ID    = 0,
        TURRET_FLYWHEEL_ID = 0;

    /**
     * Climber Motor IDs
     */
    public static final int
        CLIMBER_ID = 0;

    /**
     * Spinner IDs
     */
    public static final int
        SPINNER_ID = 5;

    /**
     * Intake IDs
     */
    public static final int
        MAININTAKE_ID = 0,
        FEEDINTAKE_ID = 0;
    
    /**
     * Feeder IDs
     */
    public static final int
        MAINFEEDER_ID = 0,
        TURRETFEEDER_ID = 0;
    
    /**
     * Drivetrain motor inverts
     */
    public static final boolean
        DRIVE_RIGHT_MASTER_INVERT = false,
        DRIVE_RIGHT_SLAVE_INVERT  = false,
        DRIVE_LEFT_MASTER_INVERT  = false,
        DRIVE_LEFT_SLAVE_INVERT   = false;

    /**
     * Turret Motor Inverts
     */
    public static final boolean
        TURRET_YAW_INVERT      = false,
        TURRET_PITCH_INVERT    = false,
        TURRET_FLYWHEEL_INVERT = false;

    /**
     * Climber Inverts
     */
    public static final boolean
        CLIMBER_INVERT = false;

    /**
     * Spinner Inverts
     */
    public static final boolean
        SPINNER_INVERT = false;
    
    /*
    * FORMAT: Red_Min, Green_Min, Blue_Min, Red_Max, Green_Max, Blue_Max.
    */
    public static final int[]
        TARGET_RED = {200,0,0,255,25,25},
        TARGET_GREEN = {0,200,0,25,255,25},
        TARGET_BLUE = {0,200,200,25,255,255},
        TARGET_YELLOW = {200,200,0,255,255,25};
  
    public static final int
        SPINNER_SPEED = 1;
}
