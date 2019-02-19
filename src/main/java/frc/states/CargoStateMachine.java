package frc.states;

public class CargoStateMachine {
    private final CargoState systemState = new CargoState();
    private IntakeState desiredIntakeState = IntakeState.STOPPED;

    public synchronized void setDesiredState(final IntakeState intakeState) {
        desiredIntakeState = intakeState;
    }

    public synchronized CargoState onUpdate(final CargoState currentState) {
        // If ball is in hold, don't allow running the intake any more
        if (currentState.ballInHold && desiredIntakeState.getStopOnSensor()) {
            systemState.intakeState = IntakeState.STOPPED;
        } else {
            systemState.intakeState = desiredIntakeState;
        }

        handleSystemStateUpdate();

        // Update whether the ball is in the hold after checking whether the systemState is equal to the current state
        // since it has no bearing on the output (we don't want to needlessly update things)
        systemState.ballInHold = currentState.ballInHold;

        return systemState;
    }

    private synchronized void handleSystemStateUpdate() {
        systemState.rearMotorOutput = systemState.intakeState.getRearMotorOutput();
        systemState.leftMotorOutput = systemState.intakeState.getLeftMotorOutput();
        systemState.rightMotorOutput = systemState.intakeState.getRightMotorOutput();
        systemState.intakeOutput = systemState.intakeState.getIntakeOutput();
    }
}
