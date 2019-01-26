package frc.subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.loops.Loop;
import frc.loops.LooperInterface;
import frc.states.ElevatorStateMachine;

import static frc.utils.Constants.*;

public class Elevator extends Subsystem {
    private static Elevator instance;
    private final CANSparkMax left;
    private final CANSparkMax right;
    private static final CANSparkMaxLowLevel.MotorType MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushed;
    private ElevatorStateMachine elevatorStateMachine = new ElevatorStateMachine();
    private ElevatorStateMachine.ElevatorState elevatorState = new ElevatorStateMachine.ElevatorState();

    private Elevator() {
        left = new CANSparkMax(LEFT_LIFT_NEO, MOTOR_TYPE);
        right = new CANSparkMax(RIGHT_LIFT_NEO, MOTOR_TYPE);
        left.setIdleMode(CANSparkMax.IdleMode.kBrake);
        right.setIdleMode(CANSparkMax.IdleMode.kBrake);
        right.setInverted(true);
        left.setInverted(true);
        right.follow(left);

        elevatorState.encoder = 0;
        elevatorState.bottomLimitTouched = false;
        elevatorState.isCargoInHold = false;
    }

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    public synchronized void setOpenLoop(double power) {
        elevatorStateMachine.setOpenLoop(power);
    }

    public synchronized void setPosition(ElevatorStateMachine.ElevatorPosition position) {
        elevatorStateMachine.setPosition(position);
    }

    @Override
    public void registerEnabledLoops(LooperInterface enabledLooper) {
        Loop loop = new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Elevator.this) {
                    stop();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Elevator.this) {
                    ElevatorStateMachine.ElevatorState newElevatorState = elevatorStateMachine.update(getUpdatedElevatorState());
                    updateOutputFromState(newElevatorState);
                }
            }

            @Override
            public void onStop(double timestamp) {
                synchronized (Elevator.this) {
                    stop();
                }
            }
        };
        enabledLooper.registerLoop(loop);
    }

    private synchronized void updateOutputFromState(ElevatorStateMachine.ElevatorState newState) {
        left.getPIDController().setReference(newState.demand, newState.controlType, 0, newState.feedforward);
        left.getPIDController().setOutputRange(newState.minimumOutput, newState.maximumOutput);
    }

    private synchronized ElevatorStateMachine.ElevatorState getUpdatedElevatorState(){
        elevatorState.isCargoInHold = false;
        elevatorState.bottomLimitTouched = false;
        elevatorState.encoder = left.getEncoder().getPosition();
        return elevatorState;
    }

    @Override
    public void outputTelemetry() {
        ELEVATOR_SHUFFLEBOARD.putString("Control Type", elevatorState.controlType.toString());
        ELEVATOR_SHUFFLEBOARD.putNumber("Demand", elevatorState.demand);
        ELEVATOR_SHUFFLEBOARD.putNumber("FeedForward", elevatorState.feedforward);
        ELEVATOR_SHUFFLEBOARD.putBoolean("Cargo In Hold", elevatorState.isCargoInHold);
        ELEVATOR_SHUFFLEBOARD.putBoolean("Bottom Limit", elevatorState.bottomLimitTouched);
        ELEVATOR_SHUFFLEBOARD.putNumber("Encoder", elevatorState.encoder);
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0.0);
    }
}
