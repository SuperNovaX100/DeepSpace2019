package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import frc.loops.Loop;
import frc.loops.LooperInterface;
import frc.utils.CheapWpiTalonSrx;
import frc.utils.Constants;
import frc.utils.DriveSignal;

import static frc.utils.Constants.*;

/**
 * The Jack subsystem, includes the three jacks and the two wheels mounted to the rear jacks.
 */
public class BlackJack extends Subsystem {
    // Motion magic parameters when lifting
    private static final int REAR_MOTION_MAGIC_VELOCITY_LIFT = 1000;
    private static final int REAR_MOTION_MAGIC_ACCELERATION_LIFT = 500;
    private static final int FRONT_MOTION_MAGIC_VELOCITY_LIFT = 1000;
    private static final int FRONT_MOTION_MAGIC_ACCELERATION_LIFT = 650;
    // Motion magic parameters when retracting
    private static final int REAR_MOTION_MAGIC_VELOCITY_RETRACT = 2000;
    private static final int REAR_MOTION_MAGIC_ACCELERATION_RETRACT = 600;
    private static final int FRONT_MOTION_MAGIC_VELOCITY_RETRACT = 2000;
    private static final int FRONT_MOTION_MAGIC_ACCELERATION_RETRACT = 600;
    private static final int LIFT_TOLERANCE = 300;
    private static final DriveSignal RETRACT_FRONT_JACK_DRIVE_BASE = new DriveSignal(0.3, 0.3);
    private static final DriveSignal RUN_DRIVE_BASE_HAB_CLIMB = new DriveSignal(0.5, 0.5);
    private static final DriveSignal RUN_JACK_WHEELS_HAB_CLIMB = new DriveSignal(1.0, 1.0);
    private static final double MAX_AMP_DRAW_ZEROING = 4.0;
    private static BlackJack instance;
    private final CheapWpiTalonSrx rightRearJack;
    private final CheapWpiTalonSrx leftRearJack;
    private final CheapWpiTalonSrx frontJack;
    private final CheapWpiTalonSrx leftRearWheel;
    private final CheapWpiTalonSrx rightRearWheel;
    private final DigitalInput forwardIrSensor;
    private final PowerDistributionPanel pdp;
    private double finishTimestamp = Timer.getFPGATimestamp();
    private static final double HOLD_REAR_AND_RUN_FORWARD_TIME = 2.0;
    private static final double HAB_CLIMB_FINISH_DRIVING_TIME = 0.4;
    private Drive drive = Drive.getInstance();
    private JackSystem state = JackSystem.OPEN_LOOP;
    private PeriodicIO periodicIo = new PeriodicIO();
    private JackControlMode jackControlMode = JackControlMode.MANUAL_CONTROL;
    private GainsState lastConfiguredGainState = null;
    private boolean frontHasZeroed = false;
    private boolean leftHasZeroed = false;
    private boolean rightHasZeroed = false;
    private JackSystem desiredState = JackSystem.INIT_HAB_CLIMB;
    private final DigitalInput rearIrSensor;

    /**
     * Constructor.
     */
    private BlackJack() {
        pdp = new PowerDistributionPanel(0);
        rightRearJack = new CheapWpiTalonSrx(Constants.RIGHT_REAR_JACK_LIFT);
        leftRearJack = new CheapWpiTalonSrx(Constants.LEFT_REAR_JACK_LIFT);
        frontJack = new CheapWpiTalonSrx(Constants.FRONT_JACK_LIFT);
        leftRearWheel = new CheapWpiTalonSrx(Constants.LEFT_REAR_JACK_WHEEL);
        rightRearWheel = new CheapWpiTalonSrx(Constants.RIGHT_REAR_JACK_WHEEL);
        leftRearWheel.setInverted(true);
        forwardIrSensor = new DigitalInput(3);
        rearIrSensor = new DigitalInput(4);
        configureTalon(rightRearJack, true, false, 1.0);
        configureTalon(leftRearJack, false, false, 1.0);
        configureTalon(frontJack, true, false, 1.7);
    }

    /**
     * Returns a static instance of the {@link BlackJack} subsystem. If none has been created yet, the instance is created.
     * This enables multiple other subsystems and any other classes to use this class without having to pass an instance
     * or take the risk of trying to instantiate multiple instances of this class, which would result in errors.
     *
     * @return a static instance of the {@link BlackJack} subsystem.
     */
    public static BlackJack getInstance() {
        if (instance == null) {
            instance = new BlackJack();
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
    private void configureTalon(WPI_TalonSRX talon, boolean inverted, boolean sensorPhase, double kp) {
        talon.setSelectedSensorPosition(0, 0, 30);
        talon.setInverted(inverted);
        talon.setSensorPhase(sensorPhase);
        talon.config_kP(0, kp, 30);
        talon.config_kI(0, 0, 30);
        talon.config_kD(0, 0, 30);
        talon.config_kF(0, 0, 30);
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
        // System.out.println(Timer.getMatchTime() + " setMotionMagicSpeedParameters");
        talon.configMotionAcceleration(acceleration);
        talon.configMotionCruiseVelocity(velocity);
    }

    /**
     * Configures the Talons' gains for lift mode, if they have not already been configured.
     */
    private void reconfigureTalonsLiftMode() {
        if (lastConfiguredGainState != GainsState.LIFT) {
            // System.out.println(Timer.getMatchTime() + " talons set to lift mode");
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
            // System.out.println(Timer.getMatchTime() + " talons set to retract mode");
            lastConfiguredGainState = GainsState.RETRACT;
            setMotionMagicSpeedParameters(frontJack, FRONT_MOTION_MAGIC_VELOCITY_RETRACT, FRONT_MOTION_MAGIC_ACCELERATION_RETRACT);
            setMotionMagicSpeedParameters(leftRearJack, REAR_MOTION_MAGIC_VELOCITY_RETRACT, REAR_MOTION_MAGIC_ACCELERATION_RETRACT);
            setMotionMagicSpeedParameters(rightRearJack, REAR_MOTION_MAGIC_VELOCITY_RETRACT, REAR_MOTION_MAGIC_ACCELERATION_RETRACT);
        }
    }

    /**
     * Resets the flags indicating whether each of the jacks have zeroed or not.
     */
    private synchronized void resetZeros() {
        // System.out.println(Timer.getMatchTime() + " reset zeros");
        frontHasZeroed = false;
        leftHasZeroed = false;
        rightHasZeroed = false;
    }

    @Override
    public void registerEnabledLoops(LooperInterface looperInterface) {
        looperInterface.registerLoop(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // do nothing
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (BlackJack.this) {
                    switch (state) {
                        case ZEROING:
                            if(zero()){
                                setState(JackSystem.STOP);
                            }
                            break;
                        case DRIVER_LIFT:
                            reconfigureTalonsLiftMode();
                            controlJacks(JackState.HAB3, JackState.HAB3, JackState.HAB3, GainsState.LIFT);
                            break;
                        case DRIVER_RETRACT:
                            reconfigureTalonsRetractMode();
                            controlJacks(JackState.RETRACT, JackState.RETRACT, JackState.RETRACT, GainsState.RETRACT);
                            break;
                        case INIT_HAB_CLIMB:
                            if(zero()){
                                setState(JackSystem.HAB_CLIMB_LIFT_ALL);
                            }
                            break;
                        case HAB_CLIMB_LIFT_ALL:
                            // TODO what if we let off of the button for a split second after we have retracted the first jack?
                            controlJacks(JackState.HAB3, JackState.HAB3, JackState.HAB3, GainsState.LIFT);
                            if (checkEncoders(LIFT_TOLERANCE)) {
                                setState(JackSystem.HAB_CLIMB_RUN_FORWARD);
                            }
                            break;
                        case HAB_CLIMB_RUN_FORWARD:
                            controlJacks(JackState.HAB3, JackState.HAB3, JackState.HAB3, GainsState.LIFT);
                            drive.setOpenLoop(RETRACT_FRONT_JACK_DRIVE_BASE);
                            setWheels(RUN_JACK_WHEELS_HAB_CLIMB);
                            if (periodicIo.frontIrDetectsGround) {
                                setState(JackSystem.HAB_CLIMB_RETRACT_FRONT_JACK);
                            }
                            break;
                        case HAB_CLIMB_RETRACT_FRONT_JACK:
                            controlJacks(JackState.RETRACT, JackState.HAB3, JackState.HAB3, GainsState.LIFT);
                            drive.setOpenLoop(RETRACT_FRONT_JACK_DRIVE_BASE);
                            setWheels(DriveSignal.NEUTRAL);
                            if (checkEncoders((int) (LIFT_TOLERANCE / 1.3))) {
                                finishTimestamp = timestamp;
                                setState(JackSystem.HAB_CLIMB_HOLD_REAR_AND_RUN_FORWARD);
                            }
                            break;
                        case HAB_CLIMB_HOLD_REAR_AND_RUN_FORWARD:
                            controlJacks(JackState.RETRACT, JackState.HAB3, JackState.HAB3, GainsState.LIFT);
                            drive.setOpenLoop(RUN_DRIVE_BASE_HAB_CLIMB);
                            setWheels(RUN_JACK_WHEELS_HAB_CLIMB);
                            if(periodicIo.rearIrDetectsGround){
//                            if(timestamp - finishTimestamp >= HOLD_REAR_AND_RUN_FORWARD_TIME/*2.0 @ 30%*/){
                                finishTimestamp = timestamp;
                                setState(JackSystem.HAB_CLIMB_RETRACT_REAR_JACKS);
                            }
                            break;
                        case HAB_CLIMB_RETRACT_REAR_JACKS:
                            controlJacks(JackState.RETRACT, JackState.RETRACT, JackState.RETRACT, GainsState.RETRACT);
                            drive.setOpenLoop(DriveSignal.NEUTRAL);
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
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    private synchronized boolean zero(){
        if (periodicIo.frontJackCurrentDraw >= MAX_AMP_DRAW_ZEROING && !frontHasZeroed) {
            frontHasZeroed = true;
            frontJack.setSelectedSensorPosition(0, 0, 30);
            periodicIo.frontJackDemand = JackState.STOP.getDemand();
            periodicIo.frontJackControlMode = JackState.STOP.getControlMode();
        }

        if (periodicIo.leftJackCurrentDraw >= MAX_AMP_DRAW_ZEROING && !leftHasZeroed) {
            leftHasZeroed = true;
            leftRearJack.setSelectedSensorPosition(0, 0, 30);
            periodicIo.leftJackDemand = JackState.STOP.getDemand();
            periodicIo.leftJackControlMode = JackState.STOP.getControlMode();
        }

        if (periodicIo.rightJackCurrentDraw >= MAX_AMP_DRAW_ZEROING && !rightHasZeroed) {
            rightHasZeroed = true;
            rightRearJack.setSelectedSensorPosition(0, 0, 30);
            periodicIo.rightJackDemand = JackState.STOP.getDemand();
            periodicIo.rightJackControlMode = JackState.STOP.getControlMode();
        }

        if (frontHasZeroed && rightHasZeroed && leftHasZeroed) {
            System.out.println("Zeroing completed");
            return true;
        }
        return false;
    }

    public synchronized void setState(JackSystem desiredState) {
        System.out.println("Desired State: " + desiredState.toString());
        if (state != desiredState) {
            state = desiredState;
            if (desiredState == JackSystem.ZEROING) {
                System.out.println("Switching to zeroing");
                resetZeros();
                periodicIo.frontJackDemand = JackState.ZEROING.getDemand();
                periodicIo.frontJackControlMode = JackState.ZEROING.getControlMode();
                periodicIo.rightJackDemand = JackState.ZEROING.getDemand();
                periodicIo.rightJackControlMode = JackState.ZEROING.getControlMode();
                periodicIo.leftJackDemand = JackState.ZEROING.getDemand();
                periodicIo.leftJackControlMode = JackState.ZEROING.getControlMode();
            } else if(desiredState == JackSystem.INIT_HAB_CLIMB){
                resetZeros();
            }
        }
    }

    public void retract() {
        setState(JackSystem.DRIVER_RETRACT);
    }

    public void lift() {
        setState(JackSystem.DRIVER_LIFT);
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
        periodicIo.frontJackDemand = front.getDemand();
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

    private double lastTimestampRead = Timer.getFPGATimestamp();

    @Override
    public synchronized void readPeriodicInputs() {
        double time = Timer.getFPGATimestamp();
        periodicIo.frontIrDetectsGround = !forwardIrSensor.get();
        periodicIo.rearIrDetectsGround = !rearIrSensor.get();
        periodicIo.frontJackEncoder = frontJack.getSelectedSensorPosition(0);
        periodicIo.leftJackEncoder = leftRearJack.getSelectedSensorPosition(0);
        periodicIo.rightJackEncoder = rightRearJack.getSelectedSensorPosition(0);

        double currentFrontVelocity = frontJack.getSelectedSensorVelocity(0);
        double currentLeftVelocity = leftRearJack.getSelectedSensorVelocity(0);
        double currentRightVelocity = rightRearJack.getSelectedSensorVelocity(0);
        double dt = time - lastTimestampRead;
        periodicIo.frontAcceleration = (currentFrontVelocity - periodicIo.frontVelocity) / dt;
        periodicIo.leftAcceleration = (currentLeftVelocity - periodicIo.leftVelocity) / dt;
        periodicIo.rightAcceleration = (currentRightVelocity - periodicIo.rightVelocity) / dt;
        periodicIo.frontVelocity = currentFrontVelocity;
        periodicIo.leftVelocity = currentLeftVelocity;
        periodicIo.rightVelocity = currentRightVelocity;

        periodicIo.frontDutyCycle = frontJack.getMotorOutputPercent();
        periodicIo.leftDutyCycle = frontJack.getMotorOutputPercent();
        periodicIo.rightDutyCycle = frontJack.getMotorOutputPercent();

        if (state == JackSystem.ZEROING) {
            periodicIo.frontJackCurrentDraw = pdp.getCurrent(FRONT_JACK_LIFT);
            periodicIo.leftJackCurrentDraw = pdp.getCurrent(LEFT_REAR_JACK_LIFT);
            periodicIo.rightJackCurrentDraw = pdp.getCurrent(RIGHT_REAR_JACK_LIFT);
        }
        lastTimestampRead = time;
    }

    /**
     * Sets all of the jack motors to a specified percent output.
     *
     * @param power the output percent to issue to all of the jacks, in the range [-1.0, 1.0].
     */
    public synchronized void setOpenLoop(double power) {
        // System.out.println(Timer.getMatchTime() + " setOpenLoop");
        setState(JackSystem.OPEN_LOOP);
        periodicIo.frontJackDemand = power;
        periodicIo.leftJackDemand = power;
        periodicIo.rightJackDemand = power;
        periodicIo.frontJackControlMode = ControlMode.PercentOutput;
        periodicIo.leftJackControlMode = ControlMode.PercentOutput;
        periodicIo.rightJackControlMode = ControlMode.PercentOutput;
    }

    public synchronized void setWheels(DriveSignal driveSignal) {
        // System.out.println(Timer.getMatchTime() + " setWheels");
        periodicIo.leftWheelDemand = driveSignal.getLeftOutput();
        periodicIo.rightWheelDemand = driveSignal.getRightOutput();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // System.out.println(Timer.getMatchTime() + " writePeriodic");
        frontJack.set(periodicIo.frontJackControlMode, periodicIo.frontJackDemand);
        leftRearJack.set(periodicIo.leftJackControlMode, periodicIo.leftJackDemand);
        rightRearJack.set(periodicIo.rightJackControlMode, periodicIo.rightJackDemand);
        leftRearWheel.set(ControlMode.PercentOutput, periodicIo.leftWheelDemand);
        rightRearWheel.set(ControlMode.PercentOutput, periodicIo.rightWheelDemand);
    }

    @Override
    public void outputTelemetry() {
        JACKS_SHUFFLEBOARD.putString("State", state.toString());
        JACKS_SHUFFLEBOARD.putNumber("Velocity Front", periodicIo.frontVelocity);
        JACKS_SHUFFLEBOARD.putNumber("Velocity Left", periodicIo.leftVelocity);
        JACKS_SHUFFLEBOARD.putNumber("Velocity Right", periodicIo.rightVelocity);
        JACKS_SHUFFLEBOARD.putNumber("Acceleration Front", periodicIo.frontAcceleration);
        JACKS_SHUFFLEBOARD.putNumber("Acceleration Left", periodicIo.leftAcceleration);
        JACKS_SHUFFLEBOARD.putNumber("Acceleration Right", periodicIo.rightAcceleration);
        JACKS_SHUFFLEBOARD.putNumber("Duty Cycle Front", periodicIo.frontDutyCycle);
        JACKS_SHUFFLEBOARD.putNumber("Duty Cycle Left", periodicIo.leftDutyCycle);
        JACKS_SHUFFLEBOARD.putNumber("Duty Cycle Right", periodicIo.rightDutyCycle);
        JACKS_SHUFFLEBOARD.putBoolean("Front Jack Zeroed", frontHasZeroed);
        JACKS_SHUFFLEBOARD.putBoolean("Left Jack Zeroed", leftHasZeroed);
        JACKS_SHUFFLEBOARD.putBoolean("Right Jack Zeroed", rightHasZeroed);
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
        HAB3(19500, ControlMode.MotionMagic),
        HAB2(5000, ControlMode.MotionMagic),
        RETRACT(0, ControlMode.MotionMagic),
        ZEROING(-0.3, ControlMode.PercentOutput),
        STOP(0.0, ControlMode.PercentOutput);

        private double demand;
        private ControlMode controlMode;

        JackState(double demand, ControlMode controlMode) {
            this.demand = demand;
            this.controlMode = controlMode;
        }

        public double getDemand() {
            return demand;
        }

        public ControlMode getControlMode() {
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

    /**
     * The control type for the jacks as a whole.
     */
    private enum JackControlMode {
        /**
         * Manually controlled mode, user manually controls the setpoints and/or power.
         */
        MANUAL_CONTROL,

        /**
         * Executes a fully autonomous climb up to the HAB platform.
         */
        HAB_CLIMB
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
        ControlMode frontJackControlMode = ControlMode.PercentOutput;
        ControlMode leftJackControlMode = ControlMode.PercentOutput;
        ControlMode rightJackControlMode = ControlMode.PercentOutput;
        double frontJackCurrentDraw;
        double leftJackCurrentDraw;
        double rightJackCurrentDraw;

        // TODO temporary
        double frontVelocity;
        double frontAcceleration;
        double leftVelocity;
        double leftAcceleration;
        double rightVelocity;
        double rightAcceleration;
        double frontDutyCycle;
        double leftDutyCycle;
        double rightDutyCycle;

        // Output
        double leftJackDemand;
        double rightJackDemand;
        double frontJackDemand;
        double leftWheelDemand;
        double rightWheelDemand;
    }
}
