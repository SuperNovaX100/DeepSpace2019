package frc.utils;

import static frc.utils.Constants.DEFAULT_NETWORK_TABLE_KEY;
import static frc.utils.Constants.USE_CUSTOM_NETWORK_TABLE_KEYS;

public class ShuffleboardConstants {
    // Network Tables
    public static final ShuffleboardWriter ROBOT_MAIN_SHUFFLEBOARD;
    public static final ShuffleboardWriter LOOPER_SHUFFLEBOARD;
    public static final ShuffleboardWriter DRIVE_SHUFFLEBOARD;
    public static final ShuffleboardWriter ELEVATOR_SHUFFLEBOARD;
    public static final ShuffleboardWriter JACKS_SHUFFLEBOARD;
    public static final ShuffleboardWriter CARGO_SHUFFLEBOARD;
    public static final ShuffleboardWriter TEST_SHUFFLEBOARD;
    public static final ShuffleboardWriter HATCH_SHUFFLEBOARD;

    static {
        if (USE_CUSTOM_NETWORK_TABLE_KEYS) {
            ROBOT_MAIN_SHUFFLEBOARD = ShuffleboardWriter.getInstance("RobotMain");
            LOOPER_SHUFFLEBOARD = ShuffleboardWriter.getInstance("Looper");
            DRIVE_SHUFFLEBOARD = ShuffleboardWriter.getInstance("Drive");
            ELEVATOR_SHUFFLEBOARD = ShuffleboardWriter.getInstance("Elevator");
            JACKS_SHUFFLEBOARD = ShuffleboardWriter.getInstance("Jacks");
            CARGO_SHUFFLEBOARD = ShuffleboardWriter.getInstance("Cargo");
            TEST_SHUFFLEBOARD = ShuffleboardWriter.getInstance("Test");
            HATCH_SHUFFLEBOARD = ShuffleboardWriter.getInstance("Hatch");
        } else {
            ROBOT_MAIN_SHUFFLEBOARD = ShuffleboardWriter.getInstance(DEFAULT_NETWORK_TABLE_KEY);
            LOOPER_SHUFFLEBOARD = ShuffleboardWriter.getInstance(DEFAULT_NETWORK_TABLE_KEY);
            DRIVE_SHUFFLEBOARD = ShuffleboardWriter.getInstance(DEFAULT_NETWORK_TABLE_KEY);
            ELEVATOR_SHUFFLEBOARD = ShuffleboardWriter.getInstance(DEFAULT_NETWORK_TABLE_KEY);
            JACKS_SHUFFLEBOARD = ShuffleboardWriter.getInstance(DEFAULT_NETWORK_TABLE_KEY);
            CARGO_SHUFFLEBOARD = ShuffleboardWriter.getInstance(DEFAULT_NETWORK_TABLE_KEY);
            TEST_SHUFFLEBOARD = ShuffleboardWriter.getInstance(DEFAULT_NETWORK_TABLE_KEY);
            HATCH_SHUFFLEBOARD = ShuffleboardWriter.getInstance(DEFAULT_NETWORK_TABLE_KEY);
        }
    }
}
