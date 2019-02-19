package frc.statemachines;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DriverStation;
import frc.states.JacksState;
import frc.subsystem.Jack0409;
import frc.utils.DriveSignal;

import static frc.utils.Constants.HAB2_ENCODER_VALUE;
import static frc.utils.Constants.HAB3_ENCODER_VALUE;

public class JacksStateMachine {
    private static final int LIFT_TOLERANCE = 300;
    private static final DriveSignal RUN_DRIVE_BASE_HAB_CLIMB = new DriveSignal(0.2, 0.2);
    private static final DriveSignal RUN_JACK_WHEELS_HAB_CLIMB = new DriveSignal(1.0, 1.0);
    private static final double MAX_AMP_DRAW_ZEROING = 4.0;
    private static final double HAB_CLIMB_FINISH_DRIVING_TIME = 0.5;
    private JacksState systemState = new JacksState();

    /**
     * Resets the flags indicating whether each of the jacks have zeroed or not.
     */
    private void resetZeros(boolean startMovingUp) {
        systemState.frontJackInput.hasZeroed = false;
        systemState.leftJackInput.hasZeroed = false;
        systemState.rightJackInput.hasZeroed = false;
        systemState.frontJackInput.currentDraw = 0.0;
        systemState.leftJackInput.currentDraw = 0.0;
        systemState.rightJackInput.currentDraw = 0.0;
        if (startMovingUp) {
            systemState.frontJackOutput.demand = JackState.ZEROING.getDemand();
            systemState.frontJackOutput.controlMode = JackState.ZEROING.getControlMode();
            systemState.leftJackOutput.demand = JackState.ZEROING.getDemand();
            systemState.leftJackOutput.controlMode = JackState.ZEROING.getControlMode();
            systemState.rightJackOutput.demand = JackState.ZEROING.getDemand();
            systemState.rightJackOutput.controlMode = JackState.ZEROING.getControlMode();
        }
    }

    private void gyroCorrect() {
        final double pitchCorrectionKp = 0.07; // %output per degree
        final double rollCorrectionKp = 0.05; // %output per degree
        final double pitchCorrectionOutput = pitchCorrectionKp * systemState.generalInput.pitch;
        final double rollCorrectionOutput = rollCorrectionKp * systemState.generalInput.roll;
        systemState.frontJackOutput.feedForward += rollCorrectionOutput;
        systemState.leftJackOutput.feedForward -= rollCorrectionOutput;
        systemState.rightJackOutput.feedForward -= rollCorrectionOutput;

        systemState.leftJackOutput.feedForward += pitchCorrectionOutput;
        systemState.rightJackOutput.feedForward -= pitchCorrectionOutput;
    }

    private boolean zero() {
        if (systemState.frontJackInput.currentDraw >= MAX_AMP_DRAW_ZEROING && !systemState.frontJackInput.hasZeroed) {
            systemState.frontJackInput.hasZeroed = true;
            systemState.frontJackInput.resetEncoder = true;
            systemState.frontJackOutput.demand = JackState.STOP.getDemand();
            systemState.frontJackOutput.controlMode = JackState.STOP.getControlMode();
        }

        if (systemState.leftJackInput.currentDraw >= MAX_AMP_DRAW_ZEROING && !systemState.leftJackInput.hasZeroed) {
            systemState.leftJackInput.hasZeroed = true;
            systemState.leftJackInput.resetEncoder = true;
            systemState.leftJackOutput.demand = JackState.STOP.getDemand();
            systemState.leftJackOutput.controlMode = JackState.STOP.getControlMode();
        }

        if (systemState.rightJackInput.currentDraw >= MAX_AMP_DRAW_ZEROING && !systemState.rightJackInput.hasZeroed) {
            systemState.rightJackInput.hasZeroed = true;
            systemState.rightJackInput.resetEncoder = true;
            systemState.rightJackOutput.demand = JackState.STOP.getDemand();
            systemState.rightJackOutput.controlMode = JackState.STOP.getControlMode();
        }

        if (systemState.frontJackInput.hasZeroed && systemState.rightJackInput.hasZeroed && systemState.leftJackInput.hasZeroed) {
            System.out.println("Zeroing completed");
            return true;
        } else {
            System.out.println("Zeroing incomplete");
        }
        return false;
    }

    private void setWheels(DriveSignal driveSignal) {
        systemState.leftWheelDemand = driveSignal.getLeftOutput();
        systemState.rightWheelDemand = driveSignal.getRightOutput();
    }

    /**
     * Checks whether all of the encoders are within tolerance of their currently set demands.
     *
     * @param tolerance the maximum tolerance for all of the motors when going to a set position.
     * @return true if all three of the jack motors are within tolerance, false if any or all of them are outside of
     * tolerance
     */
    private boolean checkEncoders(int tolerance) {
        return Math.abs(systemState.frontJackOutput.demand - systemState.frontJackInput.encoder) < tolerance
                && Math.abs(systemState.leftJackOutput.demand - systemState.leftJackInput.encoder) < tolerance
                && Math.abs(systemState.rightJackOutput.demand - systemState.rightJackInput.encoder) < tolerance;
    }

    /**
     * Sets the jacks' demands to given control modes.
     *
     * @param front      the {@link JackState} to set the front jack to.
     * @param left       the {@link JackState} to set the left jack to.
     * @param right      the {@link JackState} to set the right jack to.
     * @param gainsState the {@link Jack0409.GainsState} to use for all of the jacks.
     */
    private void controlJacks(JackState front, JackState left, JackState right, Jack0409.GainsState gainsState) {
        systemState.generalOutput.desiredGainState = gainsState;
        if (systemState.frontJackOutput.controlMode == ControlMode.MotionMagic) {
            systemState.frontJackOutput.demand = front.getDemand() + 100;
        } else {
            systemState.frontJackOutput.demand = front.getDemand();
        }
        systemState.frontJackOutput.controlMode = front.getControlMode();
        systemState.leftJackOutput.demand = left.getDemand();
        systemState.leftJackOutput.controlMode = left.getControlMode();
        systemState.rightJackOutput.demand = right.getDemand();
        systemState.rightJackOutput.controlMode = right.getControlMode();
    }

    private synchronized void setState(Jack0409.JackSystem desiredState) {
        if (systemState.generalInput.state != desiredState) {
            systemState.generalInput.state = desiredState;
            if (desiredState == Jack0409.JackSystem.ZEROING) {
                System.out.println("Switching to zeroing");
                resetZeros(true);
            } else if (desiredState == Jack0409.JackSystem.INIT_HAB_CLIMB) {
                System.out.println("Switching to init hab climb");
                resetZeros(true);
            }
        }
    }

    public JacksState update(JacksState currentState, double timestamp) {
        systemState.generalInput = currentState.generalInput;
        systemState.frontJackInput = currentState.frontJackInput;
        systemState.leftJackInput = currentState.leftJackInput;
        systemState.rightJackInput = currentState.rightJackInput;

        systemState.generalOutput.driveBaseDriveSignal = DriveSignal.NEUTRAL;
        systemState.generalOutput.resetNavXInformation = false;
        systemState.generalOutput.sendSignalToDriveBase = false;
        setState(currentState.generalInput.desiredState);
        switch (systemState.generalInput.state) {
            case ZEROING:
                if (zero()) {
                    setState(Jack0409.JackSystem.STOP);
                    resetZeros(false);
                }
                break;
            case DRIVER_LIFT:
                controlJacks(JackState.HAB3, JackState.HAB3, JackState.HAB3, Jack0409.GainsState.LIFT);
                gyroCorrect();
                break;
            case DRIVER_RETRACT:
                controlJacks(JackState.RETRACT, JackState.RETRACT, JackState.RETRACT, Jack0409.GainsState.RETRACT);
                break;
            case INIT_HAB_CLIMB:
                if (zero()) {
                    systemState.generalOutput.resetNavXInformation = true;
                    setState(Jack0409.JackSystem.HAB_CLIMB_LIFT_ALL);
                }
                break;
            case HAB_CLIMB_LIFT_ALL:
                controlJacks(systemState.generalInput.habLevelToClimbTo, systemState.generalInput.habLevelToClimbTo, systemState.generalInput.habLevelToClimbTo, Jack0409.GainsState.LIFT);
                if (checkEncoders(LIFT_TOLERANCE)) {
                    setState(Jack0409.JackSystem.HAB_CLIMB_RUN_FORWARD);
                }
                break;
            case HAB_CLIMB_RUN_FORWARD:
                controlJacks(systemState.generalInput.habLevelToClimbTo, systemState.generalInput.habLevelToClimbTo, systemState.generalInput.habLevelToClimbTo, Jack0409.GainsState.LIFT);
                systemState.generalOutput.sendSignalToDriveBase = true;
                systemState.generalOutput.driveBaseDriveSignal = DriveSignal.NEUTRAL;
                setWheels(RUN_JACK_WHEELS_HAB_CLIMB);
                if (systemState.generalInput.frontIrDetectsGround) {
                    setState(Jack0409.JackSystem.HAB_CLIMB_RETRACT_FRONT_JACK);
                }
                break;
            case HAB_CLIMB_RETRACT_FRONT_JACK:
                controlJacks(JackState.RETRACT, systemState.generalInput.habLevelToClimbTo, systemState.generalInput.habLevelToClimbTo, Jack0409.GainsState.LIFT);
                systemState.generalOutput.sendSignalToDriveBase = true;
                systemState.generalOutput.driveBaseDriveSignal = new DriveSignal(0.05, 0.05);
                setWheels(new DriveSignal(0.20, 0.20));
                if (checkEncoders((int) (LIFT_TOLERANCE / 1.3))) {
                    systemState.generalInput.finishTimestamp = timestamp;
                    setState(Jack0409.JackSystem.HAB_CLIMB_HOLD_REAR_AND_RUN_FORWARD);
                }
                break;
            case HAB_CLIMB_HOLD_REAR_AND_RUN_FORWARD:
                controlJacks(JackState.RETRACT, systemState.generalInput.habLevelToClimbTo, systemState.generalInput.habLevelToClimbTo, Jack0409.GainsState.LIFT);
                systemState.generalOutput.sendSignalToDriveBase = true;
                systemState.generalOutput.driveBaseDriveSignal = RUN_DRIVE_BASE_HAB_CLIMB;
                setWheels(RUN_JACK_WHEELS_HAB_CLIMB);
                if (systemState.generalInput.rearIrDetectsGround) {
                    systemState.generalInput.finishTimestamp = timestamp;
                    setState(Jack0409.JackSystem.HAB_CLIMB_RETRACT_REAR_JACKS);
                }
                break;
            case HAB_CLIMB_RETRACT_REAR_JACKS:
                controlJacks(JackState.RETRACT, JackState.RETRACT, JackState.RETRACT, Jack0409.GainsState.RETRACT);
                systemState.generalOutput.sendSignalToDriveBase = true;
                systemState.generalOutput.driveBaseDriveSignal = new DriveSignal(0.03, 0.03);
                setWheels(DriveSignal.NEUTRAL);
                if (checkEncoders((int) (LIFT_TOLERANCE / 1.4))) {
                    systemState.generalInput.finishTimestamp = timestamp;
                    setState(Jack0409.JackSystem.HAB_CLIMB_FINISH_DRIVING_FORWARD);
                }
                break;
            case HAB_CLIMB_FINISH_DRIVING_FORWARD:
                systemState.generalOutput.sendSignalToDriveBase = true;
                systemState.generalOutput.driveBaseDriveSignal = RUN_DRIVE_BASE_HAB_CLIMB;
                controlJacks(JackState.RETRACT, JackState.RETRACT, JackState.RETRACT, Jack0409.GainsState.RETRACT);
                if (timestamp - systemState.generalInput.finishTimestamp >= HAB_CLIMB_FINISH_DRIVING_TIME/*0.8 @ 30%*/) {
                    setState(Jack0409.JackSystem.STOP);
                }
                break;
            case OPEN_LOOP:
                controlJacks(JackState.RETRACT, JackState.RETRACT, JackState.RETRACT, Jack0409.GainsState.RETRACT);
                setWheels(DriveSignal.NEUTRAL);
                break;
            case STOP:
                controlJacks(JackState.STOP, JackState.STOP, JackState.STOP, Jack0409.GainsState.NONE);
                break;
            default:
                DriverStation.reportError("Jack state default reached", false);
        }
        return systemState;
    }


    public enum JackState {
        HAB3(HAB3_ENCODER_VALUE, ControlMode.MotionMagic),
        HAB2(HAB2_ENCODER_VALUE, ControlMode.MotionMagic),
        RETRACT(0, ControlMode.MotionMagic),
        ZEROING(-0.3, ControlMode.PercentOutput),
        STOP(0.0, ControlMode.PercentOutput);

        private final double demand;
        private final ControlMode controlMode;

        JackState(double demand, ControlMode controlMode) {
            this.demand = demand;
            this.controlMode = controlMode;
        }

        double getDemand() {
            return demand;
        }

        ControlMode getControlMode() {
            return controlMode;
        }
    }
}
