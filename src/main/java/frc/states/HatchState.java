package frc.states;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class HatchState {
    // Input
    public boolean limitHit = false;
    public int encoder = 0;
    public ControlState desiredControlState = ControlState.STOPPED;
    public double desiredDemand = 0;

    // Output
    public double demand = 0.0;
    public ControlMode controlMode = ControlMode.PercentOutput;
    public double peakOutputForward = 0.0;
    public double peakOutputReverse = 0.0;
    public boolean resetSensor = false;

    // Non-IO
    public ControlState hatchState = ControlState.STOPPED;
    public boolean hasZeroed = false;

    public enum ControlState {
        ZEROING(true),
        HATCH_PLACE_POSITION(true),
        HATCH_PULL_POSITION(true),
        OPEN_LOOP(false),
        HOLD_POSITION(false),
        STOPPED(false);

        private final boolean requiresZeroing;

        ControlState(boolean requiresZeroing) {
            this.requiresZeroing = requiresZeroing;
        }

        public boolean requiresZeroing() {
            return requiresZeroing;
        }
    }
}
