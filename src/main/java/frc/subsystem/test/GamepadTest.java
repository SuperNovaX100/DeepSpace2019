package frc.subsystem.test;

import frc.inputs.GameController;
import frc.utils.DriveSignal;

import static frc.utils.ShuffleboardConstants.TEST_SHUFFLEBOARD;

public class GamepadTest implements SubsystemTest {
    private static final String GAMEPAD_TEST_NETWORK_TABLES_PREFIX = "GamepadTest/";
    private final GameController gameController = GameController.getInstance();

    private static void putNumber(String key, double num) {
        TEST_SHUFFLEBOARD.putNumber(GAMEPAD_TEST_NETWORK_TABLES_PREFIX + key, num);
    }

    private static void putBoolean(String key, boolean booleanToPut) {
        TEST_SHUFFLEBOARD.putBoolean(GAMEPAD_TEST_NETWORK_TABLES_PREFIX + key, booleanToPut);
    }

    @Override
    public void periodic(double timestamp) {
        outputTelemetry();
    }

    @Override
    public void outputTelemetry() {
        // Drive base
        final DriveSignal driveSignal = gameController.getDriveSignal();
        putNumber("Drive/Signal Left", driveSignal.getLeftOutput());
        putNumber("Drive/Signal Right", driveSignal.getRightOutput());
        // Jacks
        putBoolean("Jacks/Lift All", gameController.liftAllJacks());
        putBoolean("Jacks/Retract All", gameController.retractAllJacks());
        putBoolean("Jacks/Initialize Hab Climbing", gameController.initializeHabClimbingLevel3());
        putBoolean("Jacks/Zero Jacks", gameController.zeroJacks());
        final DriveSignal runJackWheelsDriveSignal = gameController.runJackWheels();
        putNumber("Jacks/Wheels/Signal Left", runJackWheelsDriveSignal.getLeftOutput());
        putNumber("Jacks/Wheels/Signal Right", runJackWheelsDriveSignal.getRightOutput());
        putBoolean("Jacks/Wheels/Use Override", gameController.manualJackWheelOverride());
        // Cargo
        putBoolean("Cargo/Outtake Front", gameController.cargoOuttakeFront());
        putBoolean("Cargo/Outtake Left", gameController.cargoOuttakeLeft());
        putBoolean("Cargo/Outtake Right", gameController.cargoOuttakeRight());
        putBoolean("Cargo/Intake", gameController.cargoIntake());
        putBoolean("Cargo/Intake Left", gameController.cargoOuttakeLeft());
        putBoolean("Cargo/Intake Right", gameController.cargoOuttakeLeft());
        putNumber("Cargo/Intake Tilt", gameController.intakeTilt());
        // Elevator
        putBoolean("Elevator/Cargo Low", gameController.setElevatorPositionLowCargo());
        putBoolean("Elevator/Cargo Mid", gameController.setElevatorPositionMidCargo());
        putBoolean("Elevator/Cargo High", gameController.setElevatorPositionHighCargo());
        putBoolean("Elevator/Hatch Low", gameController.setElevatorPositionLowHatch());
        putBoolean("Elevator/Hatch Mid", gameController.setElevatorPositionMidHatch());
        putBoolean("Elevator/Hatch High", gameController.setElevatorPositionHighHatch());
        putNumber("Elevator/Manual Power", gameController.elevateManual());
        // Hatch
        putBoolean("Hatch/Use Open Loop", gameController.useHatchOpenLoop());
        putNumber("Hatch/Manual", gameController.hatchManual());
        putBoolean("Hatch/Feeder Height", gameController.placeHatch());
    }
}
