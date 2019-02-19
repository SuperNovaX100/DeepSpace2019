package frc.statemachines;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.states.HatchState;

import static frc.states.HatchState.ControlState.*;
import static frc.utils.Constants.HATCH_PLACE_ENCODER_POSITION;
import static frc.utils.Constants.HATCH_PULL_ENCODER_POSITION;

public class HatchStateMachine {
    public static final double PEAK_FORWARD_OUTPUT_STANDARD = 1.0;
    public static final double PEAK_REVERSE_OUTPUT_STANDARD = -1.0;
    static final double PEAK_FORWARD_OUTPUT_LIMIT_PRESSED = 1.0;
    static final double PEAK_REVERSE_OUTPUT_LIMIT_PRESSED = 0.0;
    static final double ZEROING_SPEED = -0.2;
    static final double HOLD_POSITION_DEADBAND = 0.04;

    private final HatchState systemState = new HatchState();
    private boolean limitHitOnLastUpdate = false;

    public HatchStateMachine() {

    }

    public HatchState update(HatchState currentState) {
        if (currentState.limitHit) {
            systemState.peakOutputForward = PEAK_FORWARD_OUTPUT_LIMIT_PRESSED;
            systemState.peakOutputReverse = PEAK_REVERSE_OUTPUT_LIMIT_PRESSED;
            if (!limitHitOnLastUpdate) {
                systemState.resetSensor = true;
                systemState.hasZeroed = true;
            } else {
                systemState.resetSensor = false;
            }
        } else {
            systemState.resetSensor = false;
            systemState.peakOutputForward = PEAK_FORWARD_OUTPUT_STANDARD;
            systemState.peakOutputReverse = PEAK_REVERSE_OUTPUT_STANDARD;
        }

        limitHitOnLastUpdate = currentState.limitHit;
        systemState.encoder = currentState.encoder;

        // If we haven't zeroed, the current state requires zeroing, and we haven't hit the limit, zero the mechanism
        if (!systemState.hasZeroed && currentState.desiredControlState.requiresZeroing()) {
            systemState.hatchState = ZEROING;
        } else if (currentState.desiredControlState == OPEN_LOOP && Math.abs(currentState.desiredDemand) < HOLD_POSITION_DEADBAND) {
            if (systemState.hatchState != HOLD_POSITION) {
                systemState.hatchState = HOLD_POSITION;
                systemState.demand = currentState.encoder;
            }
            systemState.desiredDemand = systemState.demand;
        } else {
            systemState.hatchState = currentState.desiredControlState;
            systemState.desiredDemand = currentState.desiredDemand;
        }

        systemState.limitHit = currentState.limitHit;

        handleUpdate();

        return systemState;
    }

    private void handleUpdate() {
        switch (systemState.hatchState) {
            case ZEROING:
                systemState.demand = ZEROING_SPEED;
                systemState.controlMode = ControlMode.PercentOutput;
                break;
            case HATCH_PLACE_POSITION:
                systemState.demand = HATCH_PLACE_ENCODER_POSITION;
                systemState.controlMode = ControlMode.MotionMagic;
                break;
            case HATCH_PULL_POSITION:
                systemState.demand = HATCH_PULL_ENCODER_POSITION;
                systemState.controlMode = ControlMode.MotionMagic;
                break;
            case OPEN_LOOP:
                systemState.demand = systemState.desiredDemand;
                systemState.controlMode = ControlMode.PercentOutput;
                break;
            case HOLD_POSITION:
                systemState.demand = systemState.desiredDemand;
                systemState.controlMode = ControlMode.MotionMagic;
                break;
            case STOPPED:
                systemState.demand = 0.0;
                systemState.controlMode = ControlMode.PercentOutput;
                break;
        }
    }
}
