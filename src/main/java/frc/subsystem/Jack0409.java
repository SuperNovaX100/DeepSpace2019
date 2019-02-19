package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.hardware.CheapWpiTalonSrx;
import frc.loops.Loop;
import frc.loops.LooperInterface;
import frc.statemachines.JacksStateMachine;
import frc.states.JacksState;
import frc.utils.Constants;

import static frc.utils.Constants.*;
import static frc.utils.ShuffleboardConstants.JACKS_SHUFFLEBOARD;

/**
 * The Jack subsystem, includes the three jacks and the two wheels mounted to the rear jacks.
 */
public final class Jack0409 extends Subsystem {
    // Motion magic parameters when retracting
    private static final int REAR_MOTION_MAGIC_VELOCITY_RETRACT = 2000;
    private static final int REAR_MOTION_MAGIC_ACCELERATION_RETRACT = 600;
    private static final int FRONT_MOTION_MAGIC_VELOCITY_RETRACT = 2000;
    private static final int FRONT_MOTION_MAGIC_ACCELERATION_RETRACT = 600;
    private static Jack0409 instance;
    private final CheapWpiTalonSrx rightRearJack;
    private final CheapWpiTalonSrx leftRearJack;
    private final CheapWpiTalonSrx frontJack;
    private final CheapWpiTalonSrx leftRearWheel;
    private final CheapWpiTalonSrx rightRearWheel;
    private final DigitalInput forwardIrSensor;
    private final PowerDistributionPanel pdp;
    private final DigitalInput rearIrSensor;
    private final DriveTrain drive = DriveTrain.getInstance();
    private JacksState jacksState = new JacksState();
    private JacksStateMachine jacksStateMachine = new JacksStateMachine();

    /**
     * Constructor.
     */
    private Jack0409() {
        pdp = new PowerDistributionPanel(0);
        rightRearJack = new CheapWpiTalonSrx(Constants.RIGHT_REAR_JACK_LIFT);
        leftRearJack = new CheapWpiTalonSrx(Constants.LEFT_REAR_JACK_LIFT);
        frontJack = new CheapWpiTalonSrx(Constants.FRONT_JACK_LIFT);
        leftRearWheel = new CheapWpiTalonSrx(Constants.LEFT_REAR_JACK_WHEEL);
        rightRearWheel = new CheapWpiTalonSrx(Constants.RIGHT_REAR_JACK_WHEEL);
        leftRearWheel.setInverted(true);
        forwardIrSensor = new DigitalInput(DRIVE_FRONT_IR_SENSOR);
        rearIrSensor = new DigitalInput(DRIVE_REAR_IR_SENSOR);
        configureTalon(rightRearJack, true, false, 1.0);
        configureTalon(leftRearJack, false, false, 1.0);
        configureTalon(frontJack, true, false, 1.1);
    }

    /**
     * Returns a static instance of the {@link Jack0409} subsystem. If none has been created yet, the instance is created.
     * This enables multiple other subsystems and any other classes to use this class without having to pass an instance
     * or take the risk of trying to instantiate multiple instances of this class, which would result in errors.
     *
     * @return a static instance of the {@link Jack0409} subsystem.
     */
    public static Jack0409 getInstance() {
        if (instance == null) {
            instance = new Jack0409();
        }
        return instance;
    }

    /**
     * Configures a talon and resets their gains.
     *
     * @param talon       the Talon to issue the new parameters to.
     * @param inverted    true if the motor controller should be inverted, false if it should not be.
     * @param sensorPhase false if the sensor is in phase (i.e. positive sensor change corresponds with positive motor
     *                    output, true if it is out of phase.
     * @param kp          the P gain parameter to set the Talon to use.
     */
    private void configureTalon(WPI_TalonSRX talon, boolean inverted, @SuppressWarnings("SameParameterValue") boolean sensorPhase, @SuppressWarnings("SameParameterValue") double kp) {
        talon.setSelectedSensorPosition(0, 0, SETTINGS_TIMEOUT);
        talon.setInverted(inverted);
        talon.setSensorPhase(sensorPhase);
        talon.config_kP(0, kp, SETTINGS_TIMEOUT);
        talon.config_kI(0, 0, SETTINGS_TIMEOUT);
        talon.config_kD(0, 0, SETTINGS_TIMEOUT);
        talon.config_kF(0, 0, SETTINGS_TIMEOUT);
        talon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10);
        talon.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);
        talon.configPeakOutputForward(1.0);
        talon.configPeakOutputReverse(-1.0);
    }

    /**
     * Sets the motion magic parameters for a given Talon SRX.
     *
     * @param talon        the Talon to issue the new parameters to.
     * @param velocity     the cruise velocity in sensor units per 100 milliseconds to set the controller to use.
     * @param acceleration the acceleration in sensor units per 100 milliseconds per second to set the controller to
     *                     use.
     */
    private void setMotionMagicSpeedParameters(WPI_TalonSRX talon, int velocity, int acceleration) {
        talon.configMotionAcceleration(acceleration);
        talon.configMotionCruiseVelocity(velocity);
    }

    private synchronized void reconfigureTalonsMotionMagicParameters(GainsState desiredState) {
        if (desiredState == null) {
            return;
        }

        if (jacksState.generalInput.lastConfiguredGainState != desiredState) {
            switch (desiredState) {
                case LIFT:
                    setMotionMagicSpeedParameters(frontJack, FRONT_MOTION_MAGIC_VELOCITY_LIFT, FRONT_MOTION_MAGIC_ACCELERATION_LIFT);
                    setMotionMagicSpeedParameters(leftRearJack, REAR_MOTION_MAGIC_VELOCITY_LIFT, REAR_MOTION_MAGIC_ACCELERATION_LIFT);
                    setMotionMagicSpeedParameters(rightRearJack, REAR_MOTION_MAGIC_VELOCITY_LIFT, REAR_MOTION_MAGIC_ACCELERATION_LIFT);
                    break;
                case RETRACT:
                    setMotionMagicSpeedParameters(frontJack, FRONT_MOTION_MAGIC_VELOCITY_RETRACT, FRONT_MOTION_MAGIC_ACCELERATION_RETRACT);
                    setMotionMagicSpeedParameters(leftRearJack, REAR_MOTION_MAGIC_VELOCITY_RETRACT, REAR_MOTION_MAGIC_ACCELERATION_RETRACT);
                    setMotionMagicSpeedParameters(rightRearJack, REAR_MOTION_MAGIC_VELOCITY_RETRACT, REAR_MOTION_MAGIC_ACCELERATION_RETRACT);
                    break;
                case NONE:
                    break;
            }

            if (desiredState != GainsState.NONE) {
                jacksState.generalInput.lastConfiguredGainState = desiredState;
            }
        }
    }

    public synchronized boolean controlDriveBase() {
        return jacksState.generalOutput.sendSignalToDriveBase;
    }

    @Override
    public void registerEnabledLoops(LooperInterface looperInterface) {
        looperInterface.registerLoop(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Jack0409.this) {
                    setDesiredState(JackSystem.ZEROING);
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Jack0409.this) {
                    jacksState = jacksStateMachine.update(jacksState, timestamp);
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public synchronized void beginZeroing() {
        setDesiredState(JackSystem.ZEROING);
    }

    public synchronized void beginHabClimbLevel3() {
        setDesiredState(JackSystem.INIT_HAB_CLIMB);
        jacksState.generalInput.habLevelToClimbTo = JacksStateMachine.JackState.HAB3;
    }

    public synchronized void beginHabClimbLevel2() {
        setDesiredState(JackSystem.INIT_HAB_CLIMB);
        jacksState.generalInput.habLevelToClimbTo = JacksStateMachine.JackState.HAB2;
    }

    public synchronized void retract() {
        setDesiredState(JackSystem.DRIVER_RETRACT);
    }

    public synchronized void lift() {
        setDesiredState(JackSystem.DRIVER_LIFT);
    }

    private synchronized void setDesiredState(JackSystem desiredState) {
        jacksState.generalInput.desiredState = desiredState;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        jacksState.generalInput.frontIrDetectsGround = !forwardIrSensor.get();
        jacksState.generalInput.rearIrDetectsGround = !rearIrSensor.get();
        jacksState.frontJackInput.encoder = frontJack.getSelectedSensorPosition(0);
        jacksState.leftJackInput.encoder = leftRearJack.getSelectedSensorPosition(0);
        jacksState.rightJackInput.encoder = rightRearJack.getSelectedSensorPosition(0);
        jacksState.generalInput.pitch = drive.getPitch();
        jacksState.generalInput.roll = drive.getRoll();
        jacksState.frontJackOutput.feedForward = 0.0;
        jacksState.leftJackOutput.feedForward = 0.0;
        jacksState.rightJackOutput.feedForward = 0.0;

        if (jacksState.generalInput.state == JackSystem.ZEROING || jacksState.generalInput.state == JackSystem.INIT_HAB_CLIMB) {
            jacksState.frontJackInput.currentDraw = pdp.getCurrent(FRONT_JACK_LIFT);
            jacksState.leftJackInput.currentDraw = pdp.getCurrent(LEFT_REAR_JACK_LIFT);
            jacksState.rightJackInput.currentDraw = pdp.getCurrent(RIGHT_REAR_JACK_LIFT);
        }
    }

    /**
     * Sets all of the jack motors to a specified percent output.
     *
     * @param power the output percent to issue to all of the jacks, in the range [-1.0, 1.0].
     */
    public synchronized void setOpenLoop(double power) {
        setDesiredState(JackSystem.OPEN_LOOP);
        jacksState.frontJackOutput.demand = power;
        jacksState.leftJackOutput.demand = power;
        jacksState.rightJackOutput.demand = power;
        jacksState.frontJackOutput.controlMode = ControlMode.PercentOutput;
        jacksState.leftJackOutput.controlMode = ControlMode.PercentOutput;
        jacksState.rightJackOutput.controlMode = ControlMode.PercentOutput;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (jacksState.generalOutput.resetNavXInformation) {
            drive.resetNavX();
        }

        if (jacksState.generalOutput.sendSignalToDriveBase) {
            drive.setOpenLoop(jacksState.generalOutput.driveBaseDriveSignal);
        }

        reconfigureTalonsMotionMagicParameters(jacksState.generalOutput.desiredGainState);

        if (jacksState.frontJackInput.resetEncoder) {
            frontJack.setSelectedSensorPosition(0, 0, 30);
            jacksState.frontJackInput.resetEncoder = false;
        }
        if (jacksState.leftJackInput.resetEncoder) {
            leftRearJack.setSelectedSensorPosition(0, 0, 30);
            jacksState.leftJackInput.resetEncoder = false;
        }
        if (jacksState.rightJackInput.resetEncoder) {
            rightRearJack.setSelectedSensorPosition(0, 0, 30);
            jacksState.rightJackInput.resetEncoder = false;
        }
        frontJack.set(jacksState.frontJackOutput.controlMode, jacksState.frontJackOutput.demand, DemandType.ArbitraryFeedForward, jacksState.frontJackOutput.feedForward);
        leftRearJack.set(jacksState.leftJackOutput.controlMode, jacksState.leftJackOutput.demand, DemandType.ArbitraryFeedForward, jacksState.leftJackOutput.feedForward);
        rightRearJack.set(jacksState.rightJackOutput.controlMode, jacksState.rightJackOutput.demand, DemandType.ArbitraryFeedForward, jacksState.rightJackOutput.feedForward);
        leftRearWheel.set(ControlMode.PercentOutput, jacksState.leftWheelDemand);
        rightRearWheel.set(ControlMode.PercentOutput, jacksState.rightWheelDemand);
    }

    @Override
    public synchronized void outputTelemetry() {
        JACKS_SHUFFLEBOARD.putString("State", jacksState.generalInput.state.toString());
        JACKS_SHUFFLEBOARD.putBoolean("Front Jack Zeroed", jacksState.frontJackInput.hasZeroed);
        JACKS_SHUFFLEBOARD.putBoolean("Left Jack Zeroed", jacksState.leftJackInput.hasZeroed);
        JACKS_SHUFFLEBOARD.putBoolean("Right Jack Zeroed", jacksState.rightJackInput.hasZeroed);
        JACKS_SHUFFLEBOARD.putNumber("Front Jack Demand", jacksState.frontJackOutput.demand);
        JACKS_SHUFFLEBOARD.putNumber("Left Jack Demand", jacksState.leftJackOutput.demand);
        JACKS_SHUFFLEBOARD.putNumber("Right Jack Demand", jacksState.rightJackOutput.demand);
        JACKS_SHUFFLEBOARD.putNumber("Front Jack Current", jacksState.frontJackInput.currentDraw);
        JACKS_SHUFFLEBOARD.putNumber("Left Jack Current", jacksState.leftJackInput.currentDraw);
        JACKS_SHUFFLEBOARD.putNumber("Right Jack Current", jacksState.rightJackInput.currentDraw);
        JACKS_SHUFFLEBOARD.putNumber("Right Jack Wheel Demand", jacksState.rightWheelDemand);
        JACKS_SHUFFLEBOARD.putNumber("Left Jack Wheel Demand", jacksState.leftWheelDemand);
        JACKS_SHUFFLEBOARD.putBoolean("Front Ir Sensor", jacksState.generalInput.frontIrDetectsGround);
        JACKS_SHUFFLEBOARD.putBoolean("Rear Ir Sensor", jacksState.generalInput.rearIrDetectsGround);
        JACKS_SHUFFLEBOARD.putNumber("Encoder FJ", jacksState.frontJackInput.encoder);
        JACKS_SHUFFLEBOARD.putNumber("Encoder LRF", jacksState.leftJackInput.encoder);
        JACKS_SHUFFLEBOARD.putNumber("Encoder RRJ", jacksState.rightJackInput.encoder);
    }

    @Override
    public void stop() {
        setDesiredState(JackSystem.STOP);
    }

    /**
     * The motion magic gain states since we have different gains for different modes such as lifting and retracting.
     */
    public enum GainsState {
        LIFT,
        RETRACT,
        NONE
    }

    public enum JackSystem {
        ZEROING,
        DRIVER_LIFT,
        DRIVER_RETRACT,
        INIT_HAB_CLIMB,
        HAB_CLIMB_LIFT_ALL,
        HAB_CLIMB_RUN_FORWARD,
        HAB_CLIMB_RETRACT_FRONT_JACK,
        HAB_CLIMB_HOLD_REAR_AND_RUN_FORWARD,
        HAB_CLIMB_RETRACT_REAR_JACKS,
        HAB_CLIMB_FINISH_DRIVING_FORWARD,
        OPEN_LOOP,
        STOP
    }
}