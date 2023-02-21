// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // 0.085297 is a constant that relates motor voltage to input percentage. It was experimentally determined for this motor type.
    public static final double VOLTAGE_TO_PERCENT_POWER = 0.085297;

    public final static double DRIVE_MAX_V = 4.79;

    public final static int FL_DRIVE_ID = 5;
    public final static int BL_DRIVE_ID = 7;
    public final static int BR_DRIVE_ID = 1;
    public final static int FR_DRIVE_ID = 3;

    public final static int FL_TURN_ID = 6;
    public final static int BL_TURN_ID = 8;
    public final static int BR_TURN_ID = 2;
    public final static int FR_TURN_ID = 4;

    public final static int FL_MA3_ID = 1;
    public final static int FR_MA3_ID = 2;
    public final static int BR_MA3_ID = 3;
    public final static int BL_MA3_ID = 0;

    public static final double ROTATION_DEADZONE = 0.1;
    public static final double ROTATION_COMPENSATOR_DELAY = 0.5; 

    public static final double AUTO_SPEED_LIMIT = 0.4;
    //90 - reportedAngle
    //reported angle is == what is reported with the wheel straight and the big part facing out
    /*public final static double FL_K = 90.0 - 123.93;
    public final static double FR_K = 90.0 - 328.8;
    public final static double BR_K = 90.0 - 91.8;
    public final static double BL_K = 90.0 - 203.75;*/

    public final static double CONTROLLER_DRIVE_DEADZONE = 0.1;
    public final static double FL_K = 104.587998;//-98.532904;//-158.347000;//90.0 - 197.247845;//339.726057;//340.826983;
    public final static double FR_K = 126.514780;//117.432138;//90.0 - 195.412968;
    public final static double BR_K = 75.321707;//114.218204;//90.0 - 137.342005;
    public final static double BL_K = -107.979622;//131.101973;//90.0 - 149.176963;

    public static final double SWERVE_DRIVE_MULTIPLIER = 1;
	public static final double SWERVE_TELEOP_MULTIPLIER = 1;//0.4;
	public static boolean ARE_WE_IN_TELEOP = false;

    // object manipulator constants
    public static final int ELEVATOR_ID = 9;
    public static final int EXTENDOR_ID = 10;
    public static final double ELEVATOR_MAX = 1.0795;
    public static final double ELEVATOR_MIN = 0;
    public static final double ELEVATOR_SLOW = 0.1;
    public static final double ELEVATOR_DEADZONE = 0.1;
    public static final int TOP_LIMIT_SWITCH_DIO_PIN = 0;
    public static final int BOTTOM_LIMIT_SWITCH_DIO_PIN = 1;

    public static final int PINCHER_SOLENOID_FORWARD = 2;
    public static final int PINCHER_SOLENOID_REVERSE = 3;
    public static final int PIVOT_SOLENOID_FORWARD = 1;
    public static final int PIVOT_SOLENOID_REVERSE = 0;

    // starting height in m of elevator, where 0 is all the way lowered with the bearing against the hardstop (currently set to a very rough estimate)
    public static final double ELEVATOR_START_HEIGHT = ELEVATOR_MAX;

    //pilot controller outputs
    public static final int ELEVATOR_UP = 2;
    public static final int ELEVATOR_DOWN = 3;
    public static final int CLOSE_GRABBER = 6;
    public static final int OPEN_GRABBER = 5;
    public static final int PIVOT_FORWARD = 4;
    public static final int PIVOT_BACK = 1;

    public static final int EXTEND = 7;
    public static final int RETRACT = 8;

    //copilot controllor
    public static final int MOVE_ELEVATOR_MAX = 6;
    public static final int MOVE_ELEVATOR_MIN = 5;


}
