package frc.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.subsystem.DriveTrain;
import frc.utils.DriveSignal;

public class SampleAutoBase extends AutoBase {
    private final DriveTrain drive = DriveTrain.getInstance();
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
                drive.setOpenLoop(new DriveSignal(0.25, 0.25));
//                elevator.pidToPosition(Elevator.Elevator.ElevatorPosition.CARGO_MID);
                if (timestamp - lastTimestamp > 1.0) {
                    state = State.STOP;
                }
                break;
            case STOP:
//                elevator.pidToPosition(Elevator.Elevator.ElevatorPosition.HATCH_LOW);
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
