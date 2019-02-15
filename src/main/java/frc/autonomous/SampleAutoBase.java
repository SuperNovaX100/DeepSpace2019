package frc.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.subsystem.Drive;
import frc.subsystem.Elevator;
import frc.utils.DriveSignal;

public class SampleAutoBase extends AutoBase {
    private final Drive drive = Drive.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private double lastTimestamp = Timer.getFPGATimestamp();
    private State state = State.INIT;

    @Override
    public void init() {

    }

    @Override
    public void periodic(double timestamp) {
        switch (state) {
            case INIT:
                lastTimestamp = timestamp;
                state = State.DRIVE_FORWARD_AND_LIFT_ELEVATOR;
                break;
            case DRIVE_FORWARD_AND_LIFT_ELEVATOR:
                drive.setOpenLoop(new DriveSignal(1.0, 1.0));
                elevator.pidToPosition(Elevator.ElevatorPosition.CARGO_MID);
                if (timestamp - lastTimestamp > 1.0) {
                    state = State.STOP;
                }
                break;
            case STOP:
                elevator.pidToPosition(Elevator.ElevatorPosition.HATCH_LOW);
                drive.setOpenLoop(DriveSignal.NEUTRAL);
                break;
        }
    }

    @Override
    public boolean isDone() {
        return state == State.STOP;
    }

    private enum State {
        INIT,
        DRIVE_FORWARD_AND_LIFT_ELEVATOR,
        STOP
    }
}
