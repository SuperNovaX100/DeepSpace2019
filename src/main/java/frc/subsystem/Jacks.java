package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import frc.loops.Loop;
import frc.loops.LooperInterface;
import frc.utils.CheapWpiTalonSrx;
import frc.utils.Constants;
import frc.utils.DriveSignal;

import static frc.utils.Constants.*;
import static frc.utils.ShuffleboardConstants.JACKS_SHUFFLEBOARD;

/**
 * The Jack subsystem, includes the three jacks and the two wheels mounted to the rear jacks.
 */
public final class Jacks extends Subsystem {
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
    private static Jacks instance;
    private final CheapWpiTalonSrx rightRearJack;
    private final CheapWpiTalonSrx leftRearJack;
    private final CheapWpiTalonSrx frontJack;
    private final CheapWpiTalonSrx leftRearWheel;
    private final CheapWpiTalonSrx rightRearWheel;
    private final DigitalInput forwardIrSensor;
    private final PowerDistributionPanel pdp;
    private final DigitalInput rearIrSensor;
    private final DriveTrain drive = DriveTrain.getInstance();
    private final PeriodicIO periodicIo = new PeriodicIO();
    private double finishTimestamp = Timer.getFPGATimestamp();
    private JackSystem state = JackSystem.OPEN_LOOP;
    private GainsState lastConfiguredGainState = null;
    private double lastTimestampRead = Timer.getFPGATimestamp();
    private JackState habLevelToClimbTo = JackState.RETRACT;

    /**
     * Constructor.
     */
    private Jacks() {
        pdp = new PowerDistributionPanel(0);
        rightRearJack = new CheapWpiTalonSrx(Constants.RIGHT_REAR_JACK_LIFT);
        leftRearJack = new CheapWpiTalonSrx(Constants.LEFT_REAR_JACK_LIFT);
        frontJack = new CheapWpiTalonSrx(Constants.FRONT_JACK_LIFT);
        leftRearWheel = new CheapWpiTalonSrx(Constants.LEFT_REAR_JACK_WHEEL);
        rightRearWheel = new CheapWpiTalonSrx(Constants.RIGHT_REAR_JACK_WHEEL);
        leftRearWheel.setInverted(true);
        forwardIrSensor = new DigitalInput(DRIVE_FRONT_IR_SENSOR);
        rearIrSensor = new DigitalInput(DRIVE_REAR_IR_SENSOR);
        configureTalon(rightRearJack, true, false, 1.0, 1.0, -1.0);
        configureTalon(leftRearJack, false, false, 1.0, 1.0, -1.0);
        configureTalon(frontJack, true, false, 1.1, 1.0, -1.0);
    }

    /**
     * Returns a static instance of the {@link Jacks} subsystem. If none has been created yet, the instance is created.
     * This enables multiple other subsystems and any other classes to use this class without having to pass an instance
     * or take the risk of trying to instantiate multiple instances of this class, which would result in errors.
     *
     * @return a static instance of the {@link Jacks} subsystem.
     */
    public static Jacks getInstance() {
        if (instance == null) {
            instance = new Jacks();
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
    private void configureTalon(WPI_TalonSRX talon, boolean inverted, @SuppressWarnings("SameParameterValue") boolean sensorPhase, @SuppressWarnings("SameParameterValue") double kp, @SuppressWarnings("SameParameterValue") double peakOutputForward, @SuppressWarnings("SameParameterValue") double peakOutputReverse) {
        talon.setSelectedSensorPosition(0, 0, SETTINGS_TIMEOUT);
        talon.setInverted(inverted);
        talon.setSensorPhase(sensorPhase);
        talon.config_kP(0, kp, SETTINGS_TIMEOUT);
        talon.config_kI(0, 0, SETTINGS_TIMEOUT);
        talon.config_kD(0, 0, SETTINGS_TIMEOUT);
        talon.config_kF(0, 0, SETTINGS_TIMEOUT);
        talon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10);
        talon.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);
        talon.configPeakOutputForward(peakOutputForward);
        talon.configPeakOutputReverse(peakOutputReverse);
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

    /**
     * Configures the Talons' gains for lift mode, if they have not already been configured.
     */
    private void reconfigureTalonsLiftMode() {
        if (lastConfiguredGainState != GainsState.LIFT) {
            lastConfiguredGainState = GainsState.LIFT;
            setMotionMagicSpeedParameters(frontJack, FRONT_MOTION_MAGIC_VELOCITY_LIFT, FRONT_MOTION_MAGIC_ACCELERATION_LIFT);
            setMotionMagicSpeedParameters(leftRearJack, REAR_MOTION_MAGIC_VELOCITY_LIFT, REAR_MOTION_MAGIC_ACCELERATION_LIFT);
            setMotionMagicSpeedParameters(rightRearJack, REAR_MOTION_MAGIC_VELOCITY_LIFT, REAR_MOTION_MAGIC_ACCELERATION_LIFT);
        }
    }

    /**
     * Configures the Talons' gains for retract mode, if they have not already been configured.
     */
    private void reconfigureTalonsRetractMode() {
        if (lastConfiguredGainState != GainsState.RETRACT) {
            lastConfiguredGainState = GainsState.RETRACT;
            setMotionMagicSpeedParameters(frontJack, FRONT_MOTION_MAGIC_VELOCITY_RETRACT, FRONT_MOTION_MAGIC_ACCELERATION_RETRACT);
            setMotionMagicSpeedParameters(leftRearJack, REAR_MOTION_MAGIC_VELOCITY_RETRACT, REAR_MOTION_MAGIC_ACCELERATION_RETRACT);
            setMotionMagicSpeedParameters(rightRearJack, REAR_MOTION_MAGIC_VELOCITY_RETRACT, REAR_MOTION_MAGIC_ACCELERATION_RETRACT);
        }
    }

    /**
     * Resets the flags indicating whether each of the jacks have zeroed or not.
     */
    private synchronized void resetZeros(boolean startMovingUp) {
        periodicIo.frontHasZeroed = false;
        periodicIo.leftHasZeroed = false;
        periodicIo.rightHasZeroed = false;
        periodicIo.frontJackCurrentDraw = 0.0;
        periodicIo.leftJackCurrentDraw = 0.0;
        periodicIo.rightJackCurrentDraw = 0.0;
        if (startMovingUp) {
            periodicIo.frontJackDemand = JackState.ZEROING.getDemand();
            periodicIo.frontJackControlMode = JackState.ZEROING.getControlMode();
            periodicIo.rightJackDemand = JackState.ZEROING.getDemand();
            periodicIo.rightJackControlMode = JackState.ZEROING.getControlMode();
            periodicIo.leftJackDemand = JackState.ZEROING.getDemand();
            periodicIo.leftJackControlMode = JackState.ZEROING.getControlMode();
        }
    }

    @Override
    public void registerEnabledLoops(LooperInterface looperInterface) {
        looperInterface.registerLoop(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Jacks.this) {
                    setState(JackSystem.ZEROING);
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Jacks.this) {
                    switch (state) {
                        case ZEROING:
                            if (zero()) {
                                setState(JackSystem.STOP);
                                resetZeros(false);
                            }
                            break;
                        case DRIVER_LIFT:
                            reconfigureTalonsLiftMode();
                            controlJacks(JackState.HAB3, JackState.HAB3, JackState.HAB3, GainsState.LIFT);
                            gyroCorrect();
                            break;
                        case DRIVER_RETRACT:
                            reconfigureTalonsRetractMode();
                            controlJacks(JackState.RETRACT, JackState.RETRACT, JackState.RETRACT, GainsState.RETRACT);
                            break;
                        case INIT_HAB_CLIMB:
                            if (zero()) {
                                drive.resetNavX();
                                setState(JackSystem.HAB_CLIMB_LIFT_ALL);
                            }
                            break;
                        case HAB_CLIMB_LIFT_ALL:
                            controlJacks(habLevelToClimbTo, habLevelToClimbTo, habLevelToClimbTo, GainsState.LIFT);
                            if (checkEncoders(LIFT_TOLERANCE)) {
                                setState(JackSystem.HAB_CLIMB_RUN_FORWARD);
                                leftRearJack.configPeakOutputForward(1.0);
                                rightRearJack.configPeakOutputForward(1.0);
                            }
                            break;
                        case HAB_CLIMB_RUN_FORWARD:
                            controlJacks(habLevelToClimbTo, habLevelToClimbTo, habLevelToClimbTo, GainsState.LIFT);
                            drive.setOpenLoop(DriveSignal.NEUTRAL);
                            setWheels(RUN_JACK_WHEELS_HAB_CLIMB);
                            if (periodicIo.frontIrDetectsGround) {
                                setState(JackSystem.HAB_CLIMB_RETRACT_FRONT_JACK);
                            }
                            break;
                        case HAB_CLIMB_RETRACT_FRONT_JACK:
                            controlJacks(JackState.RETRACT, habLevelToClimbTo, habLevelToClimbTo, GainsState.LIFT);
                            drive.setOpenLoop(new DriveSignal(0.05, 0.05));
                            setWheels(new DriveSignal(0.20, 0.20));
                            if (checkEncoders((int) (LIFT_TOLERANCE / 1.3))) {
                                finishTimestamp = timestamp;
                                setState(JackSystem.HAB_CLIMB_HOLD_REAR_AND_RUN_FORWARD);
                            }
                            break;
                        case HAB_CLIMB_HOLD_REAR_AND_RUN_FORWARD:
                            controlJacks(JackState.RETRACT, habLevelToClimbTo, habLevelToClimbTo, GainsState.LIFT);
                            drive.setOpenLoop(RUN_DRIVE_BASE_HAB_CLIMB);
                            setWheels(RUN_JACK_WHEELS_HAB_CLIMB);
                            if (periodicIo.rearIrDetectsGround) {
                                finishTimestamp = timestamp;
                                setState(JackSystem.HAB_CLIMB_RETRACT_REAR_JACKS);
                            }
                            break;
                        case HAB_CLIMB_RETRACT_REAR_JACKS:
                            controlJacks(JackState.RETRACT, JackState.RETRACT, JackState.RETRACT, GainsState.RETRACT);
                            drive.setOpenLoop(new DriveSignal(0.03, 0.03));
                            setWheels(DriveSignal.NEUTRAL);
                            if (checkEncoders((int) (LIFT_TOLERANCE / 1.4))) {
                                finishTimestamp = timestamp;
                                setState(JackSystem.HAB_CLIMB_FINISH_DRIVING_FORWARD);
                            }
                            break;
                        case HAB_CLIMB_FINISH_DRIVING_FORWARD:
                            drive.setOpenLoop(RUN_DRIVE_BASE_HAB_CLIMB);
                            controlJacks(JackState.RETRACT, JackState.RETRACT, JackState.RETRACT, GainsState.RETRACT);
                            if (timestamp - finishTimestamp >= HAB_CLIMB_FINISH_DRIVING_TIME/*0.8 @ 30%*/) {
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
        if (periodicIo.frontJackCurrentDraw >= MAX_AMP_DRAW_ZEROING && !periodicIo.frontHasZeroed) {
            periodicIo.frontHasZeroed = true;
            frontJack.setSelectedSensorPosition(0, 0, 30);
            periodicIo.frontJackDemand = JackState.STOP.getDemand();
            periodicIo.frontJackControlMode = JackState.STOP.getControlMode();
        }

        if (periodicIo.leftJackCurrentDraw >= MAX_AMP_DRAW_ZEROING && !periodicIo.leftHasZeroed) {
            periodicIo.leftHasZeroed = true;
            leftRearJack.setSelectedSensorPosition(0, 0, SETTINGS_TIMEOUT);
            periodicIo.leftJackDemand = JackState.STOP.getDemand();
            periodicIo.leftJackControlMode = JackState.STOP.getControlMode();
        }

        if (periodicIo.rightJackCurrentDraw >= MAX_AMP_DRAW_ZEROING && !periodicIo.rightHasZeroed) {
            periodicIo.rightHasZeroed = true;
            rightRearJack.setSelectedSensorPosition(0, 0, SETTINGS_TIMEOUT);
            periodicIo.rightJackDemand = JackState.STOP.getDemand();
            periodicIo.rightJackControlMode = JackState.STOP.getControlMode();
        }

        if (periodicIo.frontHasZeroed && periodicIo.rightHasZeroed && periodicIo.leftHasZeroed) {
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
        habLevelToClimbTo = JackState.HAB3;
    }

    public synchronized void beginHabClimbLevel2() {
        setState(JackSystem.INIT_HAB_CLIMB);
        habLevelToClimbTo = JackState.HAB2;

    }

    private synchronized void setState(JackSystem desiredState) {
        if (state != desiredState) {
            state = desiredState;
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
        final double pitchCorrectionOutput = pitchCorrectionKp * periodicIo.pitch;
        final double rollCorrectionOutput = rollCorrectionKp * periodicIo.roll;
        periodicIo.frontJackFeedForward += rollCorrectionOutput;
        periodicIo.leftJackFeedForward -= rollCorrectionOutput;
        periodicIo.rightJackFeedForward -= rollCorrectionOutput;

        periodicIo.leftJackFeedForward += pitchCorrectionOutput;
        periodicIo.rightJackFeedForward -= pitchCorrectionOutput;
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
        switch (gainsState) {
            case LIFT:
                reconfigureTalonsLiftMode();
                break;
            case RETRACT:
                reconfigureTalonsRetractMode();
                break;
            case NONE:
                break;
        }
        if (periodicIo.frontJackControlMode == ControlMode.MotionMagic) {
            periodicIo.frontJackDemand = front.getDemand() + 100;
        } else {
            periodicIo.frontJackDemand = front.getDemand();
        }
        periodicIo.frontJackControlMode = front.getControlMode();
        periodicIo.leftJackDemand = left.getDemand();
        periodicIo.leftJackControlMode = left.getControlMode();
        periodicIo.rightJackDemand = right.getDemand();
        periodicIo.rightJackControlMode = right.getControlMode();
    }

    /**
     * Checks whether all of the encoders are within tolerance of their currently set demands.
     *
     * @param tolerance the maximum tolerance for all of the motors when going to a set position.
     * @return true if all three of the jack motors are within tolerance, false if any or all of them are outside of
     * tolerance
     */
    private boolean checkEncoders(int tolerance) {
        return Math.abs(periodicIo.frontJackDemand - periodicIo.frontJackEncoder) < tolerance
                && Math.abs(periodicIo.leftJackDemand - periodicIo.leftJackEncoder) < tolerance
                && Math.abs(periodicIo.rightJackDemand - periodicIo.rightJackEncoder) < tolerance;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        periodicIo.frontIrDetectsGround = !forwardIrSensor.get();
        periodicIo.rearIrDetectsGround = !rearIrSensor.get();
        periodicIo.frontJackEncoder = frontJack.getSelectedSensorPosition(0);
        periodicIo.leftJackEncoder = leftRearJack.getSelectedSensorPosition(0);
        periodicIo.rightJackEncoder = rightRearJack.getSelectedSensorPosition(0);
        periodicIo.pitch = drive.getPitch();
        periodicIo.roll = drive.getRoll();
        periodicIo.frontJackFeedForward = 0.0;
        periodicIo.leftJackFeedForward = 0.0;
        periodicIo.rightJackFeedForward = 0.0;

        if (state == JackSystem.ZEROING || state == JackSystem.INIT_HAB_CLIMB) {
            periodicIo.frontJackCurrentDraw = pdp.getCurrent(FRONT_JACK_LIFT);
            periodicIo.leftJackCurrentDraw = pdp.getCurrent(LEFT_REAR_JACK_LIFT);
            periodicIo.rightJackCurrentDraw = pdp.getCurrent(RIGHT_REAR_JACK_LIFT);
        }
    }

    /**
     * Sets all of the jack motors to a specified percent output.
     *
     * @param power the output percent to issue to all of the jacks, in the range [-1.0, 1.0].
     */
    public synchronized void setOpenLoop(double power) {
        setState(JackSystem.OPEN_LOOP);
        periodicIo.frontJackDemand = power;
        periodicIo.leftJackDemand = power;
        periodicIo.rightJackDemand = power;
        periodicIo.frontJackControlMode = ControlMode.PercentOutput;
        periodicIo.leftJackControlMode = ControlMode.PercentOutput;
        periodicIo.rightJackControlMode = ControlMode.PercentOutput;
    }

    public synchronized void setWheels(DriveSignal driveSignal) {
        periodicIo.leftWheelDemand = driveSignal.getLeftOutput();
        periodicIo.rightWheelDemand = driveSignal.getRightOutput();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        frontJack.set(periodicIo.frontJackControlMode, periodicIo.frontJackDemand, DemandType.ArbitraryFeedForward, periodicIo.frontJackFeedForward);
        leftRearJack.set(periodicIo.leftJackControlMode, periodicIo.leftJackDemand, DemandType.ArbitraryFeedForward, periodicIo.leftJackFeedForward);
        rightRearJack.set(periodicIo.rightJackControlMode, periodicIo.rightJackDemand, DemandType.ArbitraryFeedForward, periodicIo.rightJackFeedForward);
        leftRearWheel.set(ControlMode.PercentOutput, periodicIo.leftWheelDemand);
        rightRearWheel.set(ControlMode.PercentOutput, periodicIo.rightWheelDemand);
    }

    @Override
    public synchronized void outputTelemetry() {
        JACKS_SHUFFLEBOARD.putString("State", state.toString());
        JACKS_SHUFFLEBOARD.putBoolean("Front Jack Zeroed", periodicIo.frontHasZeroed);
        JACKS_SHUFFLEBOARD.putBoolean("Left Jack Zeroed", periodicIo.leftHasZeroed);
        JACKS_SHUFFLEBOARD.putBoolean("Right Jack Zeroed", periodicIo.rightHasZeroed);
        JACKS_SHUFFLEBOARD.putNumber("Front Jack Demand", periodicIo.frontJackDemand);
        JACKS_SHUFFLEBOARD.putNumber("Left Jack Demand", periodicIo.leftJackDemand);
        JACKS_SHUFFLEBOARD.putNumber("Right Jack Demand", periodicIo.rightJackDemand);
        JACKS_SHUFFLEBOARD.putNumber("Front Jack Current", periodicIo.frontJackCurrentDraw);
        JACKS_SHUFFLEBOARD.putNumber("Left Jack Current", periodicIo.leftJackCurrentDraw);
        JACKS_SHUFFLEBOARD.putNumber("Right Jack Current", periodicIo.rightJackCurrentDraw);
        JACKS_SHUFFLEBOARD.putNumber("Right Jack Wheel Demand", periodicIo.rightWheelDemand);
        JACKS_SHUFFLEBOARD.putNumber("Left Jack Wheel Demand", periodicIo.leftWheelDemand);
        JACKS_SHUFFLEBOARD.putBoolean("Front Ir Sensor", periodicIo.frontIrDetectsGround);
        JACKS_SHUFFLEBOARD.putBoolean("Rear Ir Sensor", periodicIo.rearIrDetectsGround);
        JACKS_SHUFFLEBOARD.putNumber("Encoder RRJ", periodicIo.rightJackEncoder);
        JACKS_SHUFFLEBOARD.putNumber("Encoder LRF", periodicIo.leftJackEncoder);
        JACKS_SHUFFLEBOARD.putNumber("Encoder FJ", periodicIo.frontJackEncoder);
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
    private enum GainsState {
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

    private static class PeriodicIO {
        // Input
        boolean frontIrDetectsGround = false;
        boolean rearIrDetectsGround = false;
        double leftJackEncoder;
        double rightJackEncoder;
        double frontJackEncoder;
        boolean frontHasZeroed;
        boolean leftHasZeroed;
        boolean rightHasZeroed;
        double frontJackCurrentDraw;
        double leftJackCurrentDraw;
        double rightJackCurrentDraw;
        double pitch;
        double roll;

        // Output
        ControlMode frontJackControlMode = ControlMode.PercentOutput;
        ControlMode leftJackControlMode = ControlMode.PercentOutput;
        ControlMode rightJackControlMode = ControlMode.PercentOutput;
        double frontJackDemand;
        double leftJackDemand;
        double rightJackDemand;
        double leftWheelDemand;
        double rightWheelDemand;
        double frontJackFeedForward;
        double leftJackFeedForward;
        double rightJackFeedForward;
    }
}