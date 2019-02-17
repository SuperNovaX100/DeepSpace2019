package frc.autonomous;

import java.io.File;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystem.Drive;
import frc.subsystem.Elevator;
import frc.utils.DriveSignal;
import frc.subsystem.Hatch;
import frc.subsystem.Elevator.ElevatorPosition;
import jaci.pathfinder.*;

public class Autoline extends AutoBase {
    private final Drive drive = Drive.getInstance();
    private final Hatch hatch = Hatch.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private State state = State.INIT;

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
                elevator.pidToPosition(ElevatorPosition.HATCH_LOW);
                break;
            case EncodeTest:
                System.out.println(state);
                if (drive.isDone()){
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

    public static Trajectory loadTrajectory(String path) {
        return Pathfinder.readFromCSV(new File(path));
    }

    private enum State {
        INIT,
        EncodeTest,
        STOP
    }
}
