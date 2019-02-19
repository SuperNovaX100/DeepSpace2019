package frc.subsystem.test;

import frc.subsystem.DriveTrain;
import frc.utils.DriveSignal;

import static frc.utils.ShuffleboardConstants.TEST_SHUFFLEBOARD;

public class DriveTest implements SubsystemTest {
    private static final double BREAK_PERIOD = 0.30;
    private static final double SPEED = 0.4;
    private final DriveTrain drive = DriveTrain.getInstance();
    private DriveTestState state = DriveTestState.INIT_DISABLED;
    private double goalTimestamp = 0.0;
    private boolean initializedState = false;

    public DriveTest() {
        drive.setOpenLoop(DriveSignal.NEUTRAL);
    }

    private void neutralizeDriveBase() {
        drive.setOpenLoop(DriveSignal.NEUTRAL);
    }

    private void handleLoop(double timestamp, DriveTestState nextState) {
        if (!initializedState) {
            goalTimestamp = this.state.getTimeToRun() + timestamp;
            initializedState = true;
        }

        if (goalTimestamp + BREAK_PERIOD <= timestamp) {
            initializedState = false;
            state = nextState;
        } else if (goalTimestamp <= timestamp) {
            neutralizeDriveBase();
        }
    }

    public void outputTelemetry() {
        TEST_SHUFFLEBOARD.putString("Test State", state.toString());
        TEST_SHUFFLEBOARD.putString("Test State Expected", state.getInformation());
        drive.outputTelemetry();
    }

    public void periodic(double timestamp) {
        switch (state) {
            case INIT_DISABLED:
                handleLoop(timestamp, DriveTestState.DRIVE_FORWARD);
                break;
            case DRIVE_FORWARD:
                drive.setOpenLoop(new DriveSignal(SPEED, SPEED));
                handleLoop(timestamp, DriveTestState.TURN_RIGHT);
                break;
            case TURN_RIGHT:
                drive.setOpenLoop(new DriveSignal(SPEED, -SPEED));
                handleLoop(timestamp, DriveTestState.TURN_LEFT);
                break;
            case TURN_LEFT:
                drive.setOpenLoop(new DriveSignal(-SPEED, SPEED));
                handleLoop(timestamp, DriveTestState.DRIVE_REVERSE);
                break;
            case DRIVE_REVERSE:
                drive.setOpenLoop(new DriveSignal(-SPEED, -SPEED));
                handleLoop(timestamp, DriveTestState.END_DISABLED);
                break;
            case END_DISABLED:
                neutralizeDriveBase();
                break;
        }
        drive.writePeriodicOutputs();
    }

    private enum DriveTestState {
        INIT_DISABLED("The robot should not move before setup", 1.0),
        DRIVE_FORWARD("The robot drive forward", 2.0),
        TURN_RIGHT("The robot should be turning right", 2.0),
        TURN_LEFT("The robot should be turning left", 2.0),
        DRIVE_REVERSE("The robot should be moving in reverse", 2.0),
        END_DISABLED("The robot shouldn't be moving once disabled", 2.0);

        private final String information;
        private final double timeToRun;

        DriveTestState(String information, double timeToRun) {
            this.information = information;
            this.timeToRun = timeToRun;
        }

        String getInformation() {
            return information;
        }

        double getTimeToRun() {
            return timeToRun;
        }
    }
}
