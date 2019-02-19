package frc.utils;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.sim.mockdata.SimulatorJNI;
import frc.subsystem.DriveTrain;

import static frc.utils.UnitConversions.inchesToMeters;

public final class Constants {
    public static final double EPSILON_SMALL_DOUBLE = 1.0E-11;
    public static final int SETTINGS_TIMEOUT = 30;
    // Motor controller ports
    public static final int LEFT_DRIVE_1 = 15; // SRX
    public static final int LEFT_DRIVE_2 = 16; // SPX
    public static final int RIGHT_DRIVE_1 = 2; // SRX
    public static final int RIGHT_DRIVE_2 = 3; // SPX
    public static final int HATCH = 12; // SRX
    public static final int INTAKE_TILT = 13; // SRX
    public static final int INTAKE = 1; // SRX
    public static final int FRONT_JACK_LIFT = 11; // SRX
    public static final int RIGHT_REAR_JACK_LIFT = 7; // SRX
    public static final int LEFT_REAR_JACK_LIFT = 4; // SRX
    public static final int RIGHT_REAR_JACK_WHEEL = 6; // SRX
    public static final int LEFT_REAR_JACK_WHEEL = 5; // SRX
    public static final int ELEVATOR_NEO = 14; // MAX
    public static final int CARGO_CENTER = 10; // SRX
    public static final int CARGO_LEFT = 9; // SRX
    public static final int CARGO_RIGHT = 8; // SRX
    public static final DriverHidOption DRIVER_HID_OPTION = DriverHidOption.SINGLE_JOYSTICK;
    // HID ports
    public static final int DRIVER_JOYSTICK_PORT = 0;
    public static final int DRIVER_GAMEPAD_PORT = 0;
    public static final int OPERATOR_GAMEPAD_PORT = 1;
    public static final int DRIVER_JOYSTICK_LEFT_PORT = 0;
    public static final int DRIVER_JOYSTICK_RIGHT_PORT = 2;
    // Sensor ports
    public static final int CARGO_SENSOR = 0; // dio
    public static final int HATCH_UPPER_LIMIT_SWITCH = 1;
    public static final int ELEVATOR_BOTTOM_LIMIT = 2; // dio
    public static final int DRIVE_FRONT_IR_SENSOR = 3; // dio
    public static final int DRIVE_REAR_IR_SENSOR = 4; // dio
    public static final int CAN_TIMEOUT_MS = 10; // ms
    public static final double DRIVE_VOLTAGE_RAMP_RATE = 0; // time from neutral to full power
    public static final int HATCH_PLACE_ENCODER_POSITION;
    public static final int HATCH_PULL_ENCODER_POSITION;
    public static final boolean ELEVATOR_LIMIT_SWITCH_INVERTED;
    public static final Robot ROBOT = Robot.A_BOT;
    public static final String DEFAULT_NETWORK_TABLE_KEY = "SmartDashboard";
    public static final boolean USE_CUSTOM_NETWORK_TABLE_KEYS = true;

    public static final double TOTAL_DELTA_ENCODER_VALUE; // revolutions (given by: top value - bottom value)
    public static final double BOTTOM_DIST_FROM_GROUND;
    public static final double UPPER_DIST_FROM_GROUND;
    public static final int HAB3_ENCODER_VALUE;
    public static final int HAB2_ENCODER_VALUE;
    // Motion magic parameters when lifting
    public static final int REAR_MOTION_MAGIC_VELOCITY_LIFT;
    public static final int REAR_MOTION_MAGIC_ACCELERATION_LIFT;
    public static final int FRONT_MOTION_MAGIC_VELOCITY_LIFT;
    public static final int FRONT_MOTION_MAGIC_ACCELERATION_LIFT;
    public static final double ELEVATOR_POSITION_LOOP_KV; // (revolutions / minute) / (meter / second)

    static {
        switch (ROBOT) {
            case A_BOT:
                HATCH_PLACE_ENCODER_POSITION = 180;
                HATCH_PULL_ENCODER_POSITION = 400;
                ELEVATOR_LIMIT_SWITCH_INVERTED = true;
                TOTAL_DELTA_ENCODER_VALUE = -189.8478;
                BOTTOM_DIST_FROM_GROUND = inchesToMeters(9.0 + (1.0 / 8.0));
                UPPER_DIST_FROM_GROUND = inchesToMeters(77.5);
                HAB3_ENCODER_VALUE = 22000;
                HAB2_ENCODER_VALUE = 8500;
                REAR_MOTION_MAGIC_VELOCITY_LIFT = 1000;
                REAR_MOTION_MAGIC_ACCELERATION_LIFT = 400;
                FRONT_MOTION_MAGIC_VELOCITY_LIFT = 700;
                FRONT_MOTION_MAGIC_ACCELERATION_LIFT = 650;
                ELEVATOR_POSITION_LOOP_KV = -0.68;
                break;
            case B_BOT:
                HATCH_PLACE_ENCODER_POSITION = 470;
                HATCH_PULL_ENCODER_POSITION = 760;
                ELEVATOR_LIMIT_SWITCH_INVERTED = false;
                TOTAL_DELTA_ENCODER_VALUE = -181.22;
                BOTTOM_DIST_FROM_GROUND = inchesToMeters(9.0 + (4.0 / 8.0));
                UPPER_DIST_FROM_GROUND = inchesToMeters(74.75);
                HAB3_ENCODER_VALUE = 20000;
                HAB2_ENCODER_VALUE = 4000;/*TODO*/
                REAR_MOTION_MAGIC_VELOCITY_LIFT = 1000;
                REAR_MOTION_MAGIC_ACCELERATION_LIFT = 500;
                FRONT_MOTION_MAGIC_VELOCITY_LIFT = 1000;
                FRONT_MOTION_MAGIC_ACCELERATION_LIFT = 650;
                ELEVATOR_POSITION_LOOP_KV = -0.66;
                break;
            default:
                // A bot
                HATCH_PLACE_ENCODER_POSITION = 180;
                HATCH_PULL_ENCODER_POSITION = 400;
                ELEVATOR_LIMIT_SWITCH_INVERTED = true;
                TOTAL_DELTA_ENCODER_VALUE = -189.8478;
                BOTTOM_DIST_FROM_GROUND = inchesToMeters(9.0 + (1.0 / 8.0));
                UPPER_DIST_FROM_GROUND = inchesToMeters(77.5);
                HAB3_ENCODER_VALUE = 22000;
                HAB2_ENCODER_VALUE = 8500;
                REAR_MOTION_MAGIC_VELOCITY_LIFT = 1000;
                REAR_MOTION_MAGIC_ACCELERATION_LIFT = 400;
                FRONT_MOTION_MAGIC_VELOCITY_LIFT = 700;
                FRONT_MOTION_MAGIC_ACCELERATION_LIFT = 650;
                ELEVATOR_POSITION_LOOP_KV = -0.68;
                break;
        }
    }

    private Constants() {

    }

    public enum DriverHidOption {
        SINGLE_JOYSTICK,
        DUAL_JOYSTICKS,
        GAMEPAD
    }

    public enum Robot {
        A_BOT,
        B_BOT
    }
}