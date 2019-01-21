package frc.utils;

public class Constants {
    private Constants() {

    }

    // Motor controller ports
    public static final int LEFT_DRIVE_1 = 0; // SRX
    public static final int LEFT_DRIVE_2 = 1; // SPX
    public static final int RIGHT_DRIVE_1 = 12; // SRX
    public static final int RIGHT_DRIVE_2 = 3; // SPX
    public static final int HATCH = 2; // SRX
    public static final int INTAKE = 14; // SRX
    public static final int FRONT_JACK_LIFT = 11; // SRX
    public static final int RIGHT_REAR_JACK_LIFT = 7; // SRX
    public static final int LEFT_REAR_JACK_LIFT = 4; // SRX
    public static final int RIGHT_REAR_JACK_WHEEL = 6; // SRX
    public static final int LEFT_REAR_JACK_WHEEL = 5; // SRX
    public static final int LEFT_LIFT_NEO = 15; // MAX
    public static final int RIGHT_LIFT_NEO = 13; // MAX
    public static final int CARGO_CENTER = 10; // SRX
    public static final int CARGO_LEFT = 9; // SRX
    public static final int CARGO_RIGHT = 8; // SRX

    // HID ports

    // TODO do we even want this option?
    public static final boolean USE_JOYSTICK_FOR_DRIVING = true;
    public static final boolean USE_GAMEPAD_FOR_OPERATING = true;
    public static final int DRIVER_JOYSTICK_PORT = 0;
    public static final int DRIVER_GAMEPAD_PORT = 1;
    // TODO make these not collide
    public static final int OPERATOR_GAMEPAD_PORT = 1;
    public static final int OPERATOR_JOYSTICK_PORT = 0;


    // TODO get the actual port
    public static final int CARGO_SENSOR = 0;

    public static final String DEFAULT_NETWORK_TABLE_KEY = "SmartDashboard";
    public static final boolean USE_CUSTOM_NETWORK_TABLE_KEYS = false;
    public static final ShuffleboardWriter ROBOT_MAIN_SHUFFLEBOARD;
    public static final ShuffleboardWriter LOOPER_SHUFFLEBOARD;
    public static final ShuffleboardWriter DRIVE_SHUFFLEBOARD;
    public static final ShuffleboardWriter ELEVATOR_SHUFFLEBOARD;
    public static final ShuffleboardWriter JACKS_SHUFFLEBOARD;
    public static final ShuffleboardWriter CARGO_SHUFFLEBOARD;
    public static final ShuffleboardWriter ROBOT_STATE_SHUFFLEBOARD;

    static {
        if (USE_CUSTOM_NETWORK_TABLE_KEYS) {
            ROBOT_MAIN_SHUFFLEBOARD = ShuffleboardWriter.getInstance("RobotMain");
            LOOPER_SHUFFLEBOARD = ShuffleboardWriter.getInstance("Looper");
            DRIVE_SHUFFLEBOARD = ShuffleboardWriter.getInstance("Drive");
            ELEVATOR_SHUFFLEBOARD = ShuffleboardWriter.getInstance("Elevator");
            JACKS_SHUFFLEBOARD = ShuffleboardWriter.getInstance("Jacks");
            CARGO_SHUFFLEBOARD = ShuffleboardWriter.getInstance("Cargo");
            ROBOT_STATE_SHUFFLEBOARD = ShuffleboardWriter.getInstance("RobotState");
        } else {
            ROBOT_MAIN_SHUFFLEBOARD = ShuffleboardWriter.getInstance(DEFAULT_NETWORK_TABLE_KEY);
            LOOPER_SHUFFLEBOARD = ShuffleboardWriter.getInstance(DEFAULT_NETWORK_TABLE_KEY);
            DRIVE_SHUFFLEBOARD = ShuffleboardWriter.getInstance(DEFAULT_NETWORK_TABLE_KEY);
            ELEVATOR_SHUFFLEBOARD = ShuffleboardWriter.getInstance(DEFAULT_NETWORK_TABLE_KEY);
            JACKS_SHUFFLEBOARD = ShuffleboardWriter.getInstance(DEFAULT_NETWORK_TABLE_KEY);
            CARGO_SHUFFLEBOARD = ShuffleboardWriter.getInstance(DEFAULT_NETWORK_TABLE_KEY);
            ROBOT_STATE_SHUFFLEBOARD = ShuffleboardWriter.getInstance(DEFAULT_NETWORK_TABLE_KEY);
        }
    }
}