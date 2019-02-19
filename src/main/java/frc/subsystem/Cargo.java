package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.states.IntakeState;

import static frc.utils.Constants.*;
import static frc.utils.ShuffleboardConstants.CARGO_SHUFFLEBOARD;

public final class Cargo extends Subsystem {
    private static final double MAXIMUM_VOLTAGE = 12.0;
    private static final double MAXIMUM_INTAKE_TILT_PERCENT_OUTPUT = 0.8;
    private static Cargo instance;
    // Hardware
    private final WPI_TalonSRX centerSide;
    private final WPI_TalonSRX rightRear;
    private final WPI_TalonSRX leftRear;
    private final WPI_TalonSRX intake;
    private final WPI_TalonSRX intakeTilt;
    private final DigitalInput cargoSensor;
    private IntakeState intakeState = IntakeState.STOPPED;
    private IntakeState desiredIntakeState = IntakeState.STOPPED;
    private boolean ballInHold = false;
    private double intakeTiltOutput;

    /**
     * Constructor.
     */
    private Cargo() {
        cargoSensor = new DigitalInput(CARGO_SENSOR);
        centerSide = new WPI_TalonSRX(CARGO_CENTER);
        rightRear = new WPI_TalonSRX(CARGO_LEFT);
        leftRear = new WPI_TalonSRX(CARGO_RIGHT);
        intake = new WPI_TalonSRX(INTAKE);

        intakeTilt = new WPI_TalonSRX(INTAKE_TILT);
        intakeTilt.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        intakeTilt.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        intake.setInverted(true);
        // TODO test voltage compensation
//        centerSide.configVoltageCompSaturation(MAXIMUM_VOLTAGE, Constants.CAN_TIMEOUT_MS);
//        rightRear.configVoltageCompSaturation(MAXIMUM_VOLTAGE, Constants.CAN_TIMEOUT_MS);
//        leftRear.configVoltageCompSaturation(MAXIMUM_VOLTAGE, Constants.CAN_TIMEOUT_MS);
//        intakeTilt.configVoltageCompSaturation(MAXIMUM_VOLTAGE, Constants.CAN_TIMEOUT_MS);
//
//        centerSide.enableVoltageCompensation(true);
//        rightRear.enableVoltageCompensation(true);
//        leftRear.enableVoltageCompensation(true);
//        intake.enableVoltageCompensation(true);
//        intakeTilt.enableVoltageCompensation(true);
//
        centerSide.enableVoltageCompensation(false);
        rightRear.enableVoltageCompensation(false);
        leftRear.enableVoltageCompensation(false);
        intake.enableVoltageCompensation(false);
        intakeTilt.enableVoltageCompensation(false);

        rightRear.setInverted(true);
    }

    /**
     * Returns a static instance of the {@link Cargo} subsystem. If none has been created yet, the instance is created.
     * This enables multiple other subsystems and any other classes to use this class without having to pass an instance
     * or take the risk of trying to instantiate multiple instances of this class, which would result in errors.
     *
     * @return a static instance of the {@link Cargo} subsystem.
     */
    public synchronized static Cargo getInstance() {
        if (instance == null) {
            instance = new Cargo();
        }
        return instance;
    }

    public synchronized void setDesiredState(final IntakeState intakeState) {
        desiredIntakeState = intakeState;
    }

    // Outputs values to shuffleboard
    @Override
    public void outputTelemetry() {
        CARGO_SHUFFLEBOARD.putString("Cargo Intake State", intakeState.toString());
        CARGO_SHUFFLEBOARD.putNumber("Rear Output", intakeState.getRearMotorOutput());
        CARGO_SHUFFLEBOARD.putNumber("Left Output", intakeState.getLeftMotorOutput());
        CARGO_SHUFFLEBOARD.putNumber("Right Output", intakeState.getRightMotorOutput());
        CARGO_SHUFFLEBOARD.putNumber("Intake Output", intakeState.getIntakeOutput());
        CARGO_SHUFFLEBOARD.putNumber("Intake Tilt Output", intakeTiltOutput);
        CARGO_SHUFFLEBOARD.putBoolean("Cargo In Hold", ballInHold);
    }

    /**
     * Stops the cargo system, sets all motors in this subsystem to zero output.
     */
    @Override
    public synchronized void stop() {
        setDesiredState(IntakeState.STOPPED);
        intakeTiltOutput = 0.0;
    }

    /**
     * Returns true when there is a cargo in the holding area.
     *
     * @return true if there is a cargo in the holding area, false if there is not.
     */
    public synchronized boolean cargoInHold() {
        return ballInHold;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        ballInHold = !cargoSensor.get();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // If ball is in hold, don't allow running the intake any more
        if (ballInHold && desiredIntakeState.getStopOnSensor()) {
            intakeState = IntakeState.STOPPED;
        } else {
            intakeState = desiredIntakeState;
        }

        centerSide.set(ControlMode.PercentOutput, intakeState.getRearMotorOutput());
        rightRear.set(ControlMode.PercentOutput, intakeState.getRightMotorOutput());
        leftRear.set(ControlMode.PercentOutput, intakeState.getLeftMotorOutput());
        intake.set(ControlMode.PercentOutput, intakeState.getIntakeOutput());
        intakeTilt.set(ControlMode.PercentOutput, intakeTiltOutput);
    }

    public synchronized void intakeTilt(final double power) {
        intakeTiltOutput = MAXIMUM_INTAKE_TILT_PERCENT_OUTPUT * power;
    }
}