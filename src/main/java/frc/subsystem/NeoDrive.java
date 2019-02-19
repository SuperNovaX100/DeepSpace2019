package frc.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import frc.loops.Loop;
import frc.loops.LooperInterface;
import frc.utils.DriveSignal;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

import static frc.utils.Constants.*;
import static frc.utils.ShuffleboardConstants.DRIVE_SHUFFLEBOARD;
import static frc.utils.UnitConversions.inchesToMeters;

// TODO verify everything
// All setup variables for this subsystem
public final class NeoDrive extends DriveTrain {
    private static final double DRIVE_MASTER_DEADBAND = 0.02;
    private static final double DRIVE_WHEEL_DIAMETER = inchesToMeters(6.0);
    private static final int DRIVE_TICKS_PER_ROTATION = 4096;
    private static final double DRIVE_TICKS_PER_ROTATION_DOUBLE = DRIVE_TICKS_PER_ROTATION;
    private static final double METERS_PER_SEC_TO_TICKS_PER_100_MS = (DRIVE_TICKS_PER_ROTATION_DOUBLE / DRIVE_WHEEL_DIAMETER) * 0.1;
    // TODO remeasure on a bot
    private static final double DRIVEBASE_WIDTH = 0.56515;
    private static final Trajectory.Config CONFIG = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.01, 3, 90, 3000);
    private static NeoDrive instance;
    private final CANSparkMax leftMaster;
    private final CANSparkMax leftSlave;
    private final CANSparkMax rightMaster;
    private final CANSparkMax rightSlave;
    private final AHRS navX;
    private final CANEncoder leftEncoder;
    private final CANEncoder rightEncoder;
    private final CANPIDController leftController;
    private final CANPIDController rightController;
    private boolean isBrake = false;
    private DriveState state = DriveState.OPEN_LOOP;
    private PeriodicIO periodicIo;
    private double headingOffset = 0.0;
    private Trajectory trajectory = null;
    private Trajectory leftTrajectory = null;
    private Trajectory rightTrajectory = null;
    private int lastTrajectoryValue = 0;
    private int trajectoryValues = 0;
    private EncoderFollower leftFollower;
    private EncoderFollower rightFollower;
    private double leftEncoderOffset = 0;
    private double rightEncoderOffset = 0;
    private double yawOffset = 0.0;
    private double rollOffset = 0.0;
    private double pitchOffset = 0.0;
    private double trajectoryStartLeftPosition = 0;
    private double trajectoryStartRightPosition = 0;
    private boolean invertTrajectory = false;
    private final Loop loop = new Loop() {
        @Override
        public void onStart(final double timestamp) {
            synchronized (NeoDrive.this) {
                setOpenLoop(DriveSignal.BRAKE);
                setBrakeMode(true);
            }
        }

        @Override
        public void onLoop(final double timestamp) {
            synchronized (NeoDrive.this) {
                switch (state) {
                    case OPEN_LOOP:
                        break;
                    case PATH_FOLLOWING:
                        updatePathFollower();
                        break;
                    default:
                        DriverStation.reportError("Drive state set to: " + state + " unexpectedly", false);
                }
            }
        }

        @Override
        public void onStop(final double timestamp) {

        }
    };

    private NeoDrive() {
        periodicIo = new PeriodicIO();
        leftMaster = new CANSparkMax(LEFT_DRIVE_1, CANSparkMaxLowLevel.MotorType.kBrushless);
//        configureMaster(leftMaster, true);
        leftSlave = new CANSparkMax(LEFT_DRIVE_2, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftSlave.follow(leftMaster);

        rightMaster = new CANSparkMax(RIGHT_DRIVE_1, CANSparkMaxLowLevel.MotorType.kBrushless);
//        configureMaster(rightMaster, false);
        rightSlave = new CANSparkMax(RIGHT_DRIVE_2, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightSlave.follow(rightMaster);

        rightMaster.setInverted(true);
        rightSlave.setInverted(true);
//        leftMaster.setInverted(true);
//        leftSlave.setInverted(true);

        navX = new AHRS(SPI.Port.kMXP);

        leftEncoder = leftMaster.getEncoder();
        rightEncoder = rightMaster.getEncoder();
        leftController = leftMaster.getPIDController();
        rightController = rightMaster.getPIDController();

        setBrakeMode(true);
    }

    // Creates an instance of drive. If there is already an instance, do nothing so it doesn't conflict
    public static NeoDrive getInstance() {
        if (instance == null) {
            instance = new NeoDrive();
        }
        return instance;
    }

    // TODO change this if the gyro changes
    private static double handleGyroInput(final double heading, final double headingOffset) {
        // 0 to 360: gyroHeading = (gyroHeading + headingOffset) % 360.0;
        return (((heading + headingOffset) - 180.0) % 360.0) - 180.0;
    }

    // Determines the velocity of the robot relative to the amount of encoder ticks
    private static double wheelVelocityTicksPer100msToVelocity(final double velocityTicksPer100ms) {
        return velocityTicksPer100ms * 10.0 / DRIVE_TICKS_PER_ROTATION_DOUBLE * Math.PI * DRIVE_WHEEL_DIAMETER;
    }

    // Configures PID values for the motors
    private static void reloadGains(final WPI_TalonSRX talon, final double kP, final double kI, final double kD, final double kF, final int kIz, final int timeout) {
        talon.config_kP(0, kP, timeout);
        talon.config_kI(0, kI, timeout);
        talon.config_kD(0, kD, timeout);
        talon.config_kF(0, kF, timeout);
        talon.config_IntegralZone(0, kIz, timeout);
    }

    private void configureMaster(CANSparkMax motor, boolean left) {
        // TODO should the period be in a constant?
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 5);

        // Primary closed-loop, 100 ms timeout
        // TODO should the period be in a constant?


        // Configures the Talons with inversions and sensor values
        motor.setInverted(!left);
        motor.setInverted(true);
//        motor.enableVoltageCompensation(true);
//        motor.configVoltageCompSaturation(12.0, CAN_TIMEOUT_MS);
        // TODO tune this period
//        motor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, CAN_TIMEOUT_MS);
//        motor.configVelocityMeasurementWindow(1, CAN_TIMEOUT_MS);
//        motor.configClosedloopRamp(DRIVE_VOLTAGE_RAMP_RATE, CAN_TIMEOUT_MS);
//        motor.configNeutralDeadband(DRIVE_MASTER_DEADBAND, 0);
    }

    // Sets the motors from neutral mode coast to neutral mode brake
    public synchronized void setBrakeMode(boolean brake) {
        if (isBrake != brake) {
            CANSparkMax.IdleMode neutralMode = brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast;
            leftMaster.setIdleMode(neutralMode);
            leftSlave.setIdleMode(neutralMode);
            rightMaster.setIdleMode(neutralMode);
            rightSlave.setIdleMode(neutralMode);
            isBrake = brake;
        }
    }

    // Changes the mode to joystick input
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (state != DriveState.OPEN_LOOP) {
            setBrakeMode(true);
            // TODO should this be in a constant?
//            leftMaster.configNeutralDeadband(DRIVE_MASTER_DEADBAND, 0);
//            rightMaster.configNeutralDeadband(DRIVE_MASTER_DEADBAND, 0);
            state = DriveState.OPEN_LOOP;
        }
        periodicIo.leftDemand = signal.getLeftOutput();
        periodicIo.rightDemand = signal.getRightOutput();
        periodicIo.leftFeedForward = 0.0;
        periodicIo.rightFeedForward = 0.0;
    }

    // Puts values on the Dashboard (Shuffleboard)
    @Override
    public synchronized void outputTelemetry() {
        DRIVE_SHUFFLEBOARD.putNumber("Left Master Temp", leftMaster.getMotorTemperature());
        DRIVE_SHUFFLEBOARD.putNumber("Left Slave Temp", leftSlave.getMotorTemperature());
        DRIVE_SHUFFLEBOARD.putNumber("Right Master Temp", rightMaster.getMotorTemperature());
        DRIVE_SHUFFLEBOARD.putNumber("Right Slave Temp", rightSlave.getMotorTemperature());
        DRIVE_SHUFFLEBOARD.putNumber("Left Drive Distance (m)", periodicIo.leftDistance);
        DRIVE_SHUFFLEBOARD.putNumber("Right Drive Distance (m)", periodicIo.rightDistance);
        DRIVE_SHUFFLEBOARD.putNumber("Left Drive Ticks", periodicIo.leftPositionTicks);
        DRIVE_SHUFFLEBOARD.putNumber("Right Drive Ticks", periodicIo.rightPositionTicks);
        DRIVE_SHUFFLEBOARD.putNumber("Left Drive Distance (m)", periodicIo.leftDistance);
        DRIVE_SHUFFLEBOARD.putNumber("Right Drive Distance (m)", periodicIo.rightDistance);
        DRIVE_SHUFFLEBOARD.putNumber("Left Drive Ticks", periodicIo.leftPositionTicks);
        DRIVE_SHUFFLEBOARD.putNumber("Right Drive Ticks", periodicIo.rightPositionTicks);
        DRIVE_SHUFFLEBOARD.putNumber("Left Demand", periodicIo.leftDemand);
        DRIVE_SHUFFLEBOARD.putNumber("Right Demand", periodicIo.rightDemand);
        DRIVE_SHUFFLEBOARD.putNumber("Left Feed Forward", periodicIo.leftFeedForward);
        DRIVE_SHUFFLEBOARD.putNumber("Right Feed Forward", periodicIo.rightFeedForward);
        DRIVE_SHUFFLEBOARD.putNumber("Left Velocity Ticks Per 100 ms", periodicIo.leftVelocityTicksPer100ms);
        DRIVE_SHUFFLEBOARD.putNumber("Right Velocity Ticks Per 100 ms", periodicIo.rightVelocityTicksPer100ms);
        DRIVE_SHUFFLEBOARD.putNumber("Right Velocity", periodicIo.rightVelocity);
        DRIVE_SHUFFLEBOARD.putNumber("Left Velocity", periodicIo.leftVelocity);
        DRIVE_SHUFFLEBOARD.putNumber("Yaw", periodicIo.yaw);
        DRIVE_SHUFFLEBOARD.putNumber("Roll", periodicIo.roll);
        DRIVE_SHUFFLEBOARD.putNumber("Pitch", periodicIo.pitch);
        DRIVE_SHUFFLEBOARD.putString("State", state.toString());
    }

    // Outputs different values based on whether the robot is in teleop or autonomous
    @Override
    public synchronized void writePeriodicOutputs() {
        if (state == DriveState.OPEN_LOOP) {
            leftController.setReference(periodicIo.leftDemand, ControlType.kDutyCycle, 0, 0.0);
            rightController.setReference(periodicIo.rightDemand, ControlType.kDutyCycle, 0, 0.0);
        } else if (state == DriveState.PATH_FOLLOWING) {
            leftController.setReference(periodicIo.leftDemand, ControlType.kDutyCycle, 0, periodicIo.leftFeedForward);
            rightController.setReference(periodicIo.rightDemand, ControlType.kDutyCycle, 0, periodicIo.rightFeedForward);
        }
    }

    // Stops the robot
    @Override
    public void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    // Creates a trajectory and sets it to the motors, changing the case to Path following
    public synchronized void setTrajectory(Trajectory trajectory, boolean invert) {
        if (trajectory != null) {
            this.trajectory = trajectory;
            this.invertTrajectory = invert;
            TankModifier modifier = new TankModifier(trajectory).modify(DRIVEBASE_WIDTH);
            leftTrajectory = modifier.getLeftTrajectory();
            rightTrajectory = modifier.getRightTrajectory();
            leftFollower = new EncoderFollower(rightTrajectory);
            rightFollower = new EncoderFollower(leftTrajectory);
            trajectoryStartLeftPosition = periodicIo.leftPositionTicks;
            trajectoryStartRightPosition = periodicIo.rightPositionTicks;
            leftFollower.configureEncoder((int) (trajectoryStartLeftPosition * (4096. / 10.71)), DRIVE_TICKS_PER_ROTATION, DRIVE_WHEEL_DIAMETER);
            rightFollower.configureEncoder((int) (trajectoryStartRightPosition * (4096. / 10.71)), DRIVE_TICKS_PER_ROTATION, DRIVE_WHEEL_DIAMETER);
            leftFollower.configurePIDVA(2.0, 0.0, 0.0, 1.0 / 3.0, 0);
            rightFollower.configurePIDVA(2.0, 0.0, 0.0, 1.0 / 3.0, 0);
            lastTrajectoryValue = 0;
            trajectoryValues = trajectory.length();
            state = DriveState.PATH_FOLLOWING;
        }
    }

    // Checks for if the trajectory is completed
    public synchronized boolean isDone() {
        if (trajectory == null || state != DriveState.PATH_FOLLOWING) {
            return false;
        }
        return lastTrajectoryValue >= trajectoryValues;
    }

    // Checks for path completion and resets the Path followers for the desired velocity if not.
    private synchronized void updatePathFollower() {
        if (state == DriveState.PATH_FOLLOWING) {
            lastTrajectoryValue++;
            if (!this.isDone()) {
                int deltaLeft = (int) ((periodicIo.leftPositionTicks - trajectoryStartLeftPosition) * (4096.0 / 10.71));
                int deltaRight = (int) ((periodicIo.rightPositionTicks - trajectoryStartRightPosition) * (4096.0 / 10.71));
                final double leftPower;
                final double rightPower;
                if (!this.invertTrajectory) {
                    int leftEncoderValue = (int) (trajectoryStartLeftPosition * (4096 / 10.71)) + deltaLeft;
                    int rightEncoderValue = (int) (trajectoryStartRightPosition * (4096 / 10.71)) + deltaRight;
                    leftPower = leftFollower.calculate(leftEncoderValue);
                    rightPower = rightFollower.calculate(rightEncoderValue);
                } else {
                    int leftEncoderValue = (int) (trajectoryStartLeftPosition * (4096 / 10.71)) - deltaLeft;
                    int rightEncoderValue = (int) (trajectoryStartRightPosition * (4096 / 10.71)) - deltaRight;
                    leftPower = -leftFollower.calculate(leftEncoderValue);
                    rightPower = -rightFollower.calculate(rightEncoderValue);
                }
                DRIVE_SHUFFLEBOARD.putBoolean("Left Is Finished", leftFollower.isFinished());
                DRIVE_SHUFFLEBOARD.putBoolean("Right Is Finished", rightFollower.isFinished());
                setVelocity(new DriveSignal(leftPower, rightPower), DriveSignal.BRAKE);
            } else {
                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
            }
        } else {
            DriverStation.reportError("Drive is not in path follower mode", false);
        }
    }

    // Resets encoders
    private synchronized void resetEncoders() {
        // TODO should this be a constant
        leftEncoderOffset = -periodicIo.leftPositionTicks + leftEncoderOffset;
        rightEncoderOffset = -periodicIo.rightPositionTicks + rightEncoderOffset;
        periodicIo = new PeriodicIO();
    }

    // Configures the gyro heading
    public synchronized void setHeading(double heading) {
        headingOffset = heading - navX.getYaw();
    }

    @Override
    public void zeroSensors() {
        setHeading(0.0);
        resetEncoders();
    }

    @Override
    public void registerEnabledLoops(LooperInterface looper) {
        looper.registerLoop(loop);
    }

    // Reads the inputs from the sensors, and sets variables
    @Override
    public synchronized void readPeriodicInputs() {
        double prevLeftTicks = periodicIo.leftPositionTicks;
        double prevRightTicks = periodicIo.rightPositionTicks;
        periodicIo.leftPositionTicks = leftEncoder.getPosition() + leftEncoderOffset;
        periodicIo.rightPositionTicks = rightEncoder.getPosition() + rightEncoderOffset;
        periodicIo.leftVelocity = wheelVelocityTicksPer100msToVelocity(periodicIo.leftVelocityTicksPer100ms);
        periodicIo.rightVelocity = wheelVelocityTicksPer100msToVelocity(periodicIo.rightVelocityTicksPer100ms);
        periodicIo.yaw = navX.getYaw();
        periodicIo.roll = navX.getRoll();
        periodicIo.pitch = navX.getPitch();

        double deltaLeftTicks = ((periodicIo.leftPositionTicks - prevLeftTicks) / DRIVE_TICKS_PER_ROTATION_DOUBLE) * Math.PI;
        periodicIo.leftDistance += deltaLeftTicks * DRIVE_WHEEL_DIAMETER;

        double deltaRightTicks = ((periodicIo.rightPositionTicks - prevRightTicks) / DRIVE_TICKS_PER_ROTATION_DOUBLE) * Math.PI;
        periodicIo.rightDistance += deltaRightTicks * DRIVE_WHEEL_DIAMETER;
    }

    private synchronized void setVelocity(DriveSignal velocities, DriveSignal feedForwards) {
        if (state != DriveState.PATH_FOLLOWING) {
            setBrakeMode(true);
            // TODO should this be a constant?
//            leftMaster.selectProfileSlot(0, 0);
//            rightMaster.selectProfileSlot(0, 0);
//            leftMaster.configNeutralDeadband(DRIVE_MASTER_DEADBAND, 0);
//            rightMaster.configNeutralDeadband(DRIVE_MASTER_DEADBAND, 0);
            state = DriveState.PATH_FOLLOWING;
        }
        periodicIo.leftDemand = velocities.getLeftOutput();
        periodicIo.rightDemand = velocities.getRightOutput();
        periodicIo.leftFeedForward = feedForwards.getLeftOutput();
        periodicIo.rightFeedForward = feedForwards.getRightOutput();
    }

    public synchronized void resetNavX() {
        yawOffset = -periodicIo.yaw;
        rollOffset = -periodicIo.roll;
        pitchOffset = -periodicIo.pitch;
    }

    public synchronized double getYaw() {
        return periodicIo.yaw + yawOffset;
    }

    public synchronized double getPitch() {
        return periodicIo.pitch + pitchOffset;
    }

    public synchronized double getRoll() {
        return periodicIo.roll + rollOffset;
    }

    public synchronized double getYawRaw() {
        return periodicIo.yaw;
    }

    public synchronized double getPitchRaw() {
        return periodicIo.pitch;
    }

    public synchronized double getRollRaw() {
        return periodicIo.roll;
    }

    // List of possible states/modes
    private enum DriveState {
        OPEN_LOOP,
        PATH_FOLLOWING
    }

    // All the periodic values
    private static class PeriodicIO {
        // Input
        double leftPositionTicks;
        double rightPositionTicks;
        double leftVelocityTicksPer100ms;
        double rightVelocityTicksPer100ms;
        double leftVelocity;
        double rightVelocity;
        //    double gyroHeading;
        double leftDistance;
        double rightDistance;
        double yaw;
        double roll;
        double pitch;

//        double kP;
//        double kI;
//        double kD;
//        double kF;
//        double kIz;

        // Output
        double leftDemand;
        double rightDemand;
        double leftFeedForward;
        double rightFeedForward;
    }
}
