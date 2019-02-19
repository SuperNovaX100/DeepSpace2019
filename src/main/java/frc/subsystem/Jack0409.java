package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.hardware.CheapWpiTalonSrx;
import frc.loops.Loop;
import frc.loops.LooperInterface;
import frc.states.JacksState;
import frc.utils.Constants;
import frc.utils.DriveSignal;

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
    private static final int LIFT_TOLERANCE = 300;

    private static final DriveSignal RETRACT_FRONT_JACK_DRIVE_BASE = new DriveSignal(0.3, 0.3);
    private static final DriveSignal RUN_DRIVE_BASE_HAB_CLIMB = new DriveSignal(0.2, 0.2);

    private static final DriveSignal RUN_JACK_WHEELS_HAB_CLIMB = new DriveSignal(1.0, 1.0);
    private static final double MAX_AMP_DRAW_ZEROING = 4.0;
    private static final double HAB_CLIMB_FINISH_DRIVING_TIME = 0.5;
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
    private final JacksState jacksState = new JacksState();

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

    /**
     * Resets the flags indicating whether each of the jacks have zeroed or not.
     */
    private synchronized void resetZeros(boolean startMovingUp) {
        jacksState.frontJackInput.hasZeroed = false;
        jacksState.leftJackInput.hasZeroed = false;
        jacksState.rightJackInput.hasZeroed = false;
        jacksState.frontJackInput.currentDraw = 0.0;
        jacksState.leftJackInput.currentDraw = 0.0;
        jacksState.rightJackInput.currentDraw = 0.0;
        if (startMovingUp) {
            jacksState.frontJackOutput.demand = JackState.ZEROING.getDemand();
            jacksState.frontJackOutput.controlMode = JackState.ZEROING.getControlMode();
            jacksState.leftJackOutput.demand = JackState.ZEROING.getDemand();
            jacksState.leftJackOutput.controlMode = JackState.ZEROING.getControlMode();
            jacksState.rightJackOutput.demand = JackState.ZEROING.getDemand();
            jacksState.rightJackOutput.controlMode = JackState.ZEROING.getControlMode();
        }
    }

    public synchronized boolean controlDriveBase(){
        return jacksState.generalOutput.sendSignalToDriveBase;
    }

    @Override
    public void registerEnabledLoops(LooperInterface looperInterface) {
        looperInterface.registerLoop(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Jack0409.this) {
                    setState(JackSystem.ZEROING);
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Jack0409.this) {
                    jacksState.generalOutput.driveBaseDriveSignal = DriveSignal.NEUTRAL;
                    jacksState.generalOutput.sendSignalToDriveBase = false;
                    switch (jacksState.generalInput.state) {
                        case ZEROING:
                            if (zero()) {
                                setState(JackSystem.STOP);
                                resetZeros(false);
                            }
                            break;
                        case DRIVER_LIFT:
                            controlJacks(JackState.HAB3, JackState.HAB3, JackState.HAB3, GainsState.LIFT);
                            gyroCorrect();
                            break;
                        case DRIVER_RETRACT:
                            controlJacks(JackState.RETRACT, JackState.RETRACT, JackState.RETRACT, GainsState.RETRACT);
                            break;
                        case INIT_HAB_CLIMB:
                            if (zero()) {
                                drive.resetNavX();
                                setState(JackSystem.HAB_CLIMB_LIFT_ALL);
                            }
                            break;
                        case HAB_CLIMB_LIFT_ALL:
                            controlJacks(jacksState.generalInput.habLevelToClimbTo, jacksState.generalInput.habLevelToClimbTo, jacksState.generalInput.habLevelToClimbTo, GainsState.LIFT);
                            if (checkEncoders(LIFT_TOLERANCE)) {
                                setState(JackSystem.HAB_CLIMB_RUN_FORWARD);
                            }
                            break;
                        case HAB_CLIMB_RUN_FORWARD:
                            controlJacks(jacksState.generalInput.habLevelToClimbTo, jacksState.generalInput.habLevelToClimbTo, jacksState.generalInput.habLevelToClimbTo, GainsState.LIFT);
                            jacksState.generalOutput.sendSignalToDriveBase = true;
                            jacksState.generalOutput.driveBaseDriveSignal = DriveSignal.NEUTRAL;
                            setWheels(RUN_JACK_WHEELS_HAB_CLIMB);
                            if (jacksState.generalInput.frontIrDetectsGround) {
                                setState(JackSystem.HAB_CLIMB_RETRACT_FRONT_JACK);
                            }
                            break;
                        case HAB_CLIMB_RETRACT_FRONT_JACK:
                            controlJacks(JackState.RETRACT, jacksState.generalInput.habLevelToClimbTo, jacksState.generalInput.habLevelToClimbTo, GainsState.LIFT);
                            jacksState.generalOutput.sendSignalToDriveBase = true;
                            jacksState.generalOutput.driveBaseDriveSignal = new DriveSignal(0.05, 0.05);
                            setWheels(new DriveSignal(0.20, 0.20));
                            if (checkEncoders((int) (LIFT_TOLERANCE / 1.3))) {
                                jacksState.generalInput.finishTimestamp = timestamp;
                                setState(JackSystem.HAB_CLIMB_HOLD_REAR_AND_RUN_FORWARD);
                            }
                            break;
                        case HAB_CLIMB_HOLD_REAR_AND_RUN_FORWARD:
                            controlJacks(JackState.RETRACT, jacksState.generalInput.habLevelToClimbTo, jacksState.generalInput.habLevelToClimbTo, GainsState.LIFT);
                            jacksState.generalOutput.sendSignalToDriveBase = true;
                            jacksState.generalOutput.driveBaseDriveSignal = RUN_DRIVE_BASE_HAB_CLIMB;
                            setWheels(RUN_JACK_WHEELS_HAB_CLIMB);
                            if (jacksState.generalInput.rearIrDetectsGround) {
                                jacksState.generalInput.finishTimestamp = timestamp;
                                setState(JackSystem.HAB_CLIMB_RETRACT_REAR_JACKS);
                            }
                            break;
                        case HAB_CLIMB_RETRACT_REAR_JACKS:
                            controlJacks(JackState.RETRACT, JackState.RETRACT, JackState.RETRACT, GainsState.RETRACT);
                            jacksState.generalOutput.sendSignalToDriveBase = true;
                            jacksState.generalOutput.driveBaseDriveSignal = new DriveSignal(0.03, 0.03);
                            setWheels(DriveSignal.NEUTRAL);
                            if (checkEncoders((int) (LIFT_TOLERANCE / 1.4))) {
                                jacksState.generalInput.finishTimestamp = timestamp;
                                setState(JackSystem.HAB_CLIMB_FINISH_DRIVING_FORWARD);
                            }
                            break;
                        case HAB_CLIMB_FINISH_DRIVING_FORWARD:
                            jacksState.generalOutput.sendSignalToDriveBase = true;
                            jacksState.generalOutput.driveBaseDriveSignal = RUN_DRIVE_BASE_HAB_CLIMB;
                            controlJacks(JackState.RETRACT, JackState.RETRACT, JackState.RETRACT, GainsState.RETRACT);
                            if (timestamp - jacksState.generalInput.finishTimestamp >= HAB_CLIMB_FINISH_DRIVING_TIME/*0.8 @ 30%*/) {
                                setState(JackSystem.STOP);
                            }
                            break;
                        case OPEN_LOOP:
                            controlJacks(JackState.RETRACT, JackState.RETRACT, JackState.RETRACT, GainsState.RETRACT);
                            setWheels(DriveSignal.NEUTRAL);
                            break;
                        case STOP:
                            controlJacks(JackState.STOP, JackState.STOP, JackState.STOP, GainsState.NONE);
                            break;
                        default:
                            DriverStation.reportError("Jack state default reached", false);
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    private synchronized boolean zero() {
        if (jacksState.frontJackInput.currentDraw >= MAX_AMP_DRAW_ZEROING && !jacksState.frontJackInput.hasZeroed) {
            jacksState.frontJackInput.hasZeroed = true;
            jacksState.frontJackInput.resetEncoder = true;
            jacksState.frontJackOutput.demand = JackState.STOP.getDemand();
            jacksState.frontJackOutput.controlMode = JackState.STOP.getControlMode();
        }

        if (jacksState.leftJackInput.currentDraw >= MAX_AMP_DRAW_ZEROING && !jacksState.leftJackInput.hasZeroed) {
            jacksState.leftJackInput.hasZeroed = true;
            jacksState.leftJackInput.resetEncoder = true;
            jacksState.leftJackOutput.demand = JackState.STOP.getDemand();
            jacksState.leftJackOutput.controlMode = JackState.STOP.getControlMode();
        }

        if (jacksState.rightJackInput.currentDraw >= MAX_AMP_DRAW_ZEROING && !jacksState.rightJackInput.hasZeroed) {
            jacksState.rightJackInput.hasZeroed = true;
            jacksState.rightJackInput.resetEncoder = true;
            jacksState.rightJackOutput.demand = JackState.STOP.getDemand();
            jacksState.rightJackOutput.controlMode = JackState.STOP.getControlMode();
        }

        if (jacksState.frontJackInput.hasZeroed && jacksState.rightJackInput.hasZeroed && jacksState.leftJackInput.hasZeroed) {
            System.out.println("Zeroing completed");
            return true;
        } else {
            System.out.println("Zeroing incomplete");
        }
        return false;
    }

    public synchronized void beginZeroing() {
        setState(JackSystem.ZEROING);
    }

    public synchronized void beginHabClimbLevel3() {
        setState(JackSystem.INIT_HAB_CLIMB);
        jacksState.generalInput.habLevelToClimbTo = JackState.HAB3;
    }

    public synchronized void beginHabClimbLevel2() {
        setState(JackSystem.INIT_HAB_CLIMB);
        jacksState.generalInput.habLevelToClimbTo = JackState.HAB2;
    }

    private synchronized void setState(JackSystem desiredState) {
        if (jacksState.generalInput.state != desiredState) {
            jacksState.generalInput.state = desiredState;
            if (desiredState == JackSystem.ZEROING) {
                System.out.println("Switching to zeroing");
                resetZeros(true);
            } else if (desiredState == JackSystem.INIT_HAB_CLIMB) {
                System.out.println("Switching to init hab climb");
                resetZeros(true);
            }
        }
    }

    public synchronized void retract() {
        setState(JackSystem.DRIVER_RETRACT);
    }

    public synchronized void lift() {
        setState(JackSystem.DRIVER_LIFT);
    }

    private synchronized void gyroCorrect() {
        final double pitchCorrectionKp = 0.07; // %output per degree
        final double rollCorrectionKp = 0.05; // %output per degree
        final double pitchCorrectionOutput = pitchCorrectionKp * jacksState.generalInput.pitch;
        final double rollCorrectionOutput = rollCorrectionKp * jacksState.generalInput.roll;
        jacksState.frontJackOutput.feedForward += rollCorrectionOutput;
        jacksState.leftJackOutput.feedForward -= rollCorrectionOutput;
        jacksState.rightJackOutput.feedForward -= rollCorrectionOutput;

        jacksState.leftJackOutput.feedForward += pitchCorrectionOutput;
        jacksState.rightJackOutput.feedForward -= pitchCorrectionOutput;
    }

    /**
     * Sets the jacks' demands to given control modes.
     *
     * @param front      the {@link JackState} to set the front jack to.
     * @param left       the {@link JackState} to set the left jack to.
     * @param right      the {@link JackState} to set the right jack to.
     * @param gainsState the {@link GainsState} to use for all of the jacks.
     */
    private synchronized void controlJacks(JackState front, JackState left, JackState right, GainsState gainsState) {
        jacksState.generalOutput.desiredGainState = gainsState;
        if (jacksState.frontJackOutput.controlMode == ControlMode.MotionMagic) {
            jacksState.frontJackOutput.demand = front.getDemand() + 100;
        } else {
            jacksState.frontJackOutput.demand = front.getDemand();
        }
        jacksState.frontJackOutput.controlMode = front.getControlMode();
        jacksState.leftJackOutput.demand = left.getDemand();
        jacksState.leftJackOutput.controlMode = left.getControlMode();
        jacksState.rightJackOutput.demand = right.getDemand();
        jacksState.rightJackOutput.controlMode = right.getControlMode();
    }

    /**
     * Checks whether all of the encoders are within tolerance of their currently set demands.
     *
     * @param tolerance the maximum tolerance for all of the motors when going to a set position.
     * @return true if all three of the jack motors are within tolerance, false if any or all of them are outside of
     * tolerance
     */
    private boolean checkEncoders(int tolerance) {
        return Math.abs(jacksState.frontJackOutput.demand - jacksState.frontJackInput.encoder) < tolerance
                && Math.abs(jacksState.leftJackOutput.demand - jacksState.leftJackInput.encoder) < tolerance
                && Math.abs(jacksState.rightJackOutput.demand - jacksState.rightJackInput.encoder) < tolerance;
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
        setState(JackSystem.OPEN_LOOP);
        jacksState.frontJackOutput.demand = power;
        jacksState.leftJackOutput.demand = power;
        jacksState.rightJackOutput.demand = power;
        jacksState.frontJackOutput.controlMode = ControlMode.PercentOutput;
        jacksState.leftJackOutput.controlMode = ControlMode.PercentOutput;
        jacksState.rightJackOutput.controlMode = ControlMode.PercentOutput;
    }

    public synchronized void setWheels(DriveSignal driveSignal) {
        jacksState.leftWheelDemand = driveSignal.getLeftOutput();
        jacksState.rightWheelDemand = driveSignal.getRightOutput();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
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
        setState(JackSystem.STOP);
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