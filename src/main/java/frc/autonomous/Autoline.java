package frc.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystem.DriveTrain;
import frc.subsystem.Elevator;
import frc.subsystem.Hatch;
import frc.utils.DriveSignal;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

import java.io.File;

public class Autoline extends AutoBase {
    private final DriveTrain drive = DriveTrain.getInstance();
    private final Hatch hatch = Hatch.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private State state = State.INIT;

    public static Trajectory loadTrajectory(String path) {
        return Pathfinder.readFromCSV(new File(path));
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic(double timestamp) {
        SmartDashboard.putString("Autostate", state.toString());
        switch (state) {
            case INIT:
                System.out.println(state);
                state = State.EncodeTest;
                drive.setTrajectory(loadTrajectory("/home/lvuser/deploy/paths/EncodeTest.pf1.csv"), false);
                elevator.pidToPosition(Elevator.ElevatorPosition.HATCH_LOW);
                break;
            case EncodeTest:
                System.out.println(state);
                if (drive.isDone()) {
                    state = State.STOP;
                }
                break;
            case STOP:
                System.out.println(state);
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
        EncodeTest,
        STOP
    }
}
