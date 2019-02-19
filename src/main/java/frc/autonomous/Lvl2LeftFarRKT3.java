package frc.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystem.DriveTrain;
import frc.subsystem.Elevator;
import frc.subsystem.Hatch;
import frc.utils.DriveSignal;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

import java.io.File;

public class Lvl2LeftFarRKT3 extends AutoBase {
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
                state = State.LEVEL2_to_RocketLEFT_Far;
                drive.setTrajectory(loadTrajectory("/home/lvuser/deploy/paths/LEVEL2_to_RocketLEFT_Far.pf1.csv"), false);
                elevator.pidToPosition(Elevator.ElevatorPosition.HATCH_HIGH);
                System.out.println(state);
                break;
            case LEVEL2_to_RocketLEFT_Far:
                hatch.setHatchPlace();
                if (drive.isDone()) {
                    state = State.Left_RKT_Far_Backup;
                    System.out.println(state);
                    drive.setTrajectory(loadTrajectory("/home/lvuser/deploy/paths/Left_RKT_Far_Backup.pf1.csv"), true);
                }
                break;
            case Left_RKT_Far_Backup:
                hatch.setHatchPull();
                if (drive.isDone()) {
                    state = State.Far_Left_Rkt_to_FEED;
                    System.out.println(state);
                    drive.setTrajectory(loadTrajectory("/home/lvuser/deploy/paths/Far_Left_Rkt_to_FEED.pf1.csv"), false);

                }
                break;
            case Far_Left_Rkt_to_FEED:
                if (drive.isDone()) {
                    state = State.STOP;
                    System.out.println(state);
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
        LEVEL2_to_RocketLEFT_Far,
        Left_RKT_Far_Backup,
        Far_Left_Rkt_to_FEED,
        STOP
    }
}
