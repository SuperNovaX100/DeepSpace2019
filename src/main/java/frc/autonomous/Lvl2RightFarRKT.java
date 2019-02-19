package frc.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystem.DriveTrain;
import frc.subsystem.Elevator;
import frc.subsystem.Hatch;
import frc.utils.DriveSignal;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

import java.io.File;

public class Lvl2RightFarRKT extends AutoBase {
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
                state = State.LEVEL2_to_Rocket_Far;
                drive.setTrajectory(loadTrajectory("/home/lvuser/deploy/paths/LEVEL2_to_Rocket_Far.path.pf1.csv"), false);
                elevator.pidToPosition(Elevator.ElevatorPosition.HATCH_LOW);
                System.out.println(state);
                break;
            case LEVEL2_to_Rocket_Far:
                hatch.setHatchPlace();
                if (drive.isDone()) {
                    state = State.Right_RKT_Far_Backup;
                    System.out.println(state);
                    drive.setTrajectory(loadTrajectory("/home/lvuser/deploy/paths/Right_RKT_Far_Backup.pf1.csv"), true);
                }
                break;
            case Right_RKT_Far_Backup:
                hatch.setHatchPull();
                if (drive.isDone()) {
                    state = State.Far_Right_Rkt_to_FEED;
                    System.out.println(state);
                    drive.setTrajectory(loadTrajectory("/home/lvuser/deploy/paths/Far_Right_Rkt_to_FEED.pf1.csv"), false);

                }
                break;
            //180 degree turn
            case Far_Right_Rkt_to_FEED:
                if (drive.isDone()) {
                    state = State.STOP;
                    System.out.println(state);
                }
                break;
            case STOP:
                System.out.println(state);
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
        LEVEL2_to_Rocket_Far,
        Right_RKT_Far_Backup,
        Far_Right_Rkt_to_FEED,
        STOP
    }
}
