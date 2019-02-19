package frc.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.loops.Loop;
import frc.loops.Looper;
import frc.loops.LooperInterface;
import frc.states.HatchState;
import frc.statemachines.HatchStateMachine;

import static frc.statemachines.HatchStateMachine.PEAK_FORWARD_OUTPUT_STANDARD;
import static frc.statemachines.HatchStateMachine.PEAK_REVERSE_OUTPUT_STANDARD;
import static frc.utils.Constants.*;

public final class Hatch extends Subsystem {
    private static final int PERIOD_MS = (int) (Looper.PERIOD * 1000);
    private static final double EPSILON = 1.0E-5;
    private static Hatch instance;
    private final WPI_TalonSRX hatch;
    private final DigitalInput upperLimitSwitch;
    private final HatchStateMachine hatchStateMachine = new HatchStateMachine();
    private final HatchState hatchState = new HatchState();
    private HatchState outputState = new HatchState();

    private Hatch() {
        upperLimitSwitch = new DigitalInput(HATCH_UPPER_LIMIT_SWITCH);
        hatch = new WPI_TalonSRX(HATCH);

        hatch.configFactoryDefault();

        hatch.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, SETTINGS_TIMEOUT);
        hatch.setInverted(false);
        hatch.setSensorPhase(false);

        hatch.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, PERIOD_MS, SETTINGS_TIMEOUT);
        hatch.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, PERIOD_MS, SETTINGS_TIMEOUT);

        hatch.configNominalOutputReverse(0.0, SETTINGS_TIMEOUT);
        hatch.configNominalOutputForward(0.0, SETTINGS_TIMEOUT);
        hatch.configPeakOutputForward(PEAK_FORWARD_OUTPUT_STANDARD);
        hatch.configPeakOutputReverse(PEAK_REVERSE_OUTPUT_STANDARD);

        hatch.config_kF(0, 0.0); //todo
        hatch.config_kP(0, 2.0); //todo
        hatch.config_kI(0, 0.0); //todo
        hatch.config_kD(0, 0.0); //todo
        // TODO hatch.config_IntegralZone(0, 30);

        hatch.configMotionCruiseVelocity(6000, SETTINGS_TIMEOUT);
        hatch.configMotionAcceleration(6000, SETTINGS_TIMEOUT);

        resetSensorPosition();
    }

    public static Hatch getInstance() {
        if (instance == null) {
            instance = new Hatch();
        }
        return instance;
    }

    @Override
    public void registerEnabledLoops(LooperInterface enabledLooper) {
        Loop loop = new Loop() {
            @Override
            public void onStart(double timestamp) {
                stop();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Hatch.this) {
                    outputState = hatchStateMachine.update(hatchState);
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        };

        enabledLooper.registerLoop(loop);
    }

    private synchronized void resetSensorPosition() {
        hatch.setSelectedSensorPosition(0, 0, SETTINGS_TIMEOUT);
    }

    public synchronized void setOpenLoop(double power) {
        hatchStateMachine.setOpenLoop(-power);
    }

    public synchronized void setHatchPlace() {
        hatchStateMachine.setHatchPlace();
    }

    public synchronized void setHatchPull() {
        hatchStateMachine.setHatchPull();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        hatchState.limitHit = !upperLimitSwitch.get();
        hatchState.encoder = hatch.getSelectedSensorPosition(0);
//        hatchState.ensureHatchIsOut = elevator.ensureHatchStatysOut();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (outputState.resetSensor) {
            resetSensorPosition();
        }
        if (Math.abs(outputState.peakOutputReverse - hatchState.peakOutputReverse) > EPSILON) {
            hatchState.peakOutputReverse = outputState.peakOutputReverse;
            hatch.configPeakOutputReverse(outputState.peakOutputReverse, SETTINGS_TIMEOUT);
        }
        if (Math.abs(outputState.peakOutputForward - hatchState.peakOutputForward) > EPSILON) {
            hatchState.peakOutputForward = outputState.peakOutputForward;
            hatch.configPeakOutputForward(outputState.peakOutputForward, SETTINGS_TIMEOUT);
        }
        hatch.set(outputState.controlMode, outputState.demand);
    }

    @Override
    public synchronized void outputTelemetry() {
        HATCH_SHUFFLEBOARD.putBoolean("Limit Hit", hatchState.limitHit);
        HATCH_SHUFFLEBOARD.putNumber("Demand", outputState.demand);
        HATCH_SHUFFLEBOARD.putNumber("Peak Output Forward", hatchState.peakOutputForward);
        HATCH_SHUFFLEBOARD.putNumber("Peak Output Reverse", hatchState.peakOutputReverse);
        HATCH_SHUFFLEBOARD.putString("Control Mode", hatchState.controlMode.toString());
        HATCH_SHUFFLEBOARD.putNumber("Encoder", hatchState.encoder);
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0.0);
    }
}
