package frc.utils;

import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import java.util.Optional;

/**
 * Wrapper around all three of the {@link CANPIDController}, {@link ResettableCanEncoder}, and {@link CANSparkMax} to
 * encapsulate all of their functions into one class in a similar way to how the
 * {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX} is packaged.
 */
public class SparkMaxNeoContoller extends CANSparkMax {
    private final ResettableCanEncoder encoder;
    private final CANPIDController controller;

    /**
     * Create a new SPARK MAX Controller
     *
     * @param deviceID The device ID.
     * @param type     The motor type connected to the controller. Brushless motors
     *                 must be connected to their matching color and the hall sensor
     *                 plugged in. Brushed motors must be connected to the Red and
     */
    public SparkMaxNeoContoller(int deviceID, MotorType type) {
        super(deviceID, type);
        encoder = new ResettableCanEncoder(this);
        controller = super.getPIDController();
    }

    /**
     * Get the position of the motor. This returns the native units
     * of 'rotations' by default, and can be changed by a scale factor
     * using setPositionConversionFactor().
     *
     * @return Number of rotations of the motor
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * Get the velocity of the motor. This returns the native units
     * of 'RPM' by default, and can be changed by a scale factor
     * using setVelocityConversionFactor().
     *
     * @return Number the RPM of the motor
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }


    /**
     * Set the position of the encoder.  By default the units
     * are 'rotations' and can be changed by a scale factor
     * using setPositionConversionFactor().
     *
     * @param position Number of rotations of the motor
     * @return CANError Set to CANError.kOk if successful
     */
    public CANError setPosition(double position) {
        return encoder.setPosition(position);
    }

    /**
     * Set the conversion factor for position of the encoder.
     * Multiplied by the native output units to give you position.
     *
     * @param factor The conversion factor to multiply the native units by
     * @return CANError Set to CANError.kOk if successful
     */
    public CANError setPositionConversionFactor(double factor) {
        return encoder.setPositionConversionFactor(factor);
    }

    /**
     * Set the conversion factor for velocity of the encoder.
     * Multiplied by the native output units to give you velocity
     *
     * @param factor The conversion factor to multiply the native units by
     * @return CANError Set to CANError.kOk if successful
     */
    public CANError setVelocityConversionFactor(double factor) {
        return encoder.setVelocityConversionFactor(factor);
    }

    /**
     * Get the conversion factor for position of the encoder.
     * Multiplied by the native output units to give you position
     *
     * @return The conversion factor for position
     */
    public double getPositionConversionFactor() {
        return encoder.getPositionConversionFactor();
    }

    /**
     * Get the conversion factor for velocity of the encoder.
     * Multiplied by the native output units to give you velocity
     *
     * @return The conversion factor for velocity
     */
    public double getVelocityConversionFactor() {
        return encoder.getVelocityConversionFactor();
    }

    public void resetEncoder() {
        encoder.resetEncoder();
    }

    public void resetEncoder(double desiredCurrentValue) {
        encoder.resetEncoder(desiredCurrentValue);
    }

    /**
     * Set the controller reference value based on the selected control mode.
     *
     * @param value The value to set depending on the control mode. For basic
     *              duty cycle control this should be a value between -1 and 1
     *              Otherwise: Voltage Control: Voltage (volts) Velocity Control: Velocity
     *              (RPM) Position Control: Position (Rotations) Current Control: Current
     *              (Amps). Native units can be changed using the setPositionConversionFactor()
     *              or setVelocityConversionFactor() methods of the CANEncoder class
     * @param ctrl  Is the control type
     * @return CANError Set to REV_OK if successful
     */
    public CANError setReference(double value, ControlType ctrl) {
        return controller.setReference(value, ctrl);
    }

    /**
     * Set the controller reference value based on the selected control mode.
     * This will override the pre-programmed control mode but not change what
     * is programmed to the controller.
     *
     * @param value   The value to set depending on the control mode. For basic
     *                duty cycle control this should be a value between -1 and 1
     *                Otherwise: Voltage Control: Voltage (volts) Velocity Control: Velocity
     *                (RPM) Position Control: Position (Rotations) Current Control: Current
     *                (Amps). Native units can be changed using the setPositionConversionFactor()
     *                or setVelocityConversionFactor() methods of the CANEncoder class
     * @param ctrl    Is the control type to override with
     * @param pidSlot for this command
     * @return CANError Set to REV_OK if successful
     */
    public CANError setReference(double value, ControlType ctrl, int pidSlot) {
        return controller.setReference(value, ctrl, pidSlot);
    }

    /**
     * Set the controller reference value based on the selected control mode.
     * This will override the pre-programmed control mode but not change what
     * is programmed to the controller.
     *
     * @param value          The value to set depending on the control mode. For basic
     *                       duty cycle control this should be a value between -1 and 1
     *                       Otherwise: Voltage Control: Voltage (volts) Velocity Control: Velocity
     *                       (RPM) Position Control: Position (Rotations) Current Control: Current
     *                       (Amps). Native units can be changed using the setPositionConversionFactor()
     *                       or setVelocityConversionFactor() methods of the CANEncoder class
     * @param ctrl           Is the control type to override with
     * @param pidSlot        for this command
     * @param arbFeedforward A value from which is represented in voltage
     *                       applied to the motor after the result of the specified control mode. The
     *                       units for the parameter is Volts. This value is set after the control mode,
     *                       but before any current limits or ramp rates.
     * @return CANError Set to REV_OK if successful
     */
    public CANError setReference(double value, ControlType ctrl, int pidSlot,
                                 double arbFeedforward) {
        return controller.setReference(value, ctrl, pidSlot, arbFeedforward);
    }

    /**
     * Set the Proportional Gain constant of the PIDF controller on the SPARK
     * MAX. This uses the Set Parameter API and should be used infrequently. The
     * parameter does not presist unless burnFlash() is called.  The recommended
     * method to configure this parameter is use to SPARK MAX GUI to tune and
     * save parameters.
     *
     * @param gain The proportional gain value, must be positive
     * @return CANError Set to REV_OK if successful
     */
    public CANError setP(double gain) {
        return controller.setP(gain);
    }

    /**
     * Set the Proportional Gain constant of the PIDF controller on the SPARK
     * MAX. This uses the Set Parameter API and should be used infrequently. The
     * parameter does not presist unless burnFlash() is called.  The recommended
     * method to configure this parameter is use to SPARK MAX GUI to tune and
     * save parameters.
     *
     * @param gain   The proportional gain value, must be positive
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return CANError Set to REV_OK if successful
     */
    public CANError setP(double gain, int slotID) {
        return controller.setP(gain, slotID);
    }

    /**
     * Set the Integral Gain constant of the PIDF controller on the SPARK MAX.
     * This uses the Set Parameter API and should be used infrequently. The
     * parameter does not presist unless burnFlash() is called.  The recommended
     * method to configure this parameter is use to SPARK MAX GUI to tune and
     * save parameters.
     *
     * @param gain The integral gain value, must be positive
     * @return CANError Set to REV_OK if successful
     */
    public CANError setI(double gain) {
        return controller.setI(gain);
    }

    /**
     * Set the Integral Gain constant of the PIDF controller on the SPARK MAX.
     * This uses the Set Parameter API and should be used infrequently. The
     * parameter does not presist unless burnFlash() is called.  The recommended
     * method to configure this parameter is use to SPARK MAX GUI to tune and
     * save parameters.
     *
     * @param gain   The integral gain value, must be positive
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return CANError Set to REV_OK if successful
     */
    public CANError setI(double gain, int slotID) {
        return controller.setI(gain, slotID);
    }

    /**
     * Set the Derivative Gain constant of the PIDF controller on the SPARK MAX.
     * This uses the Set Parameter API and should be used infrequently. The
     * parameter does not presist unless burnFlash() is called.  The recommended
     * method to configure this parameter is use to SPARK MAX GUI to tune and
     * save parameters.
     *
     * @param gain The derivative gain value, must be positive
     * @return CANError Set to REV_OK if successful
     */
    public CANError setD(double gain) {
        return controller.setD(gain);
    }

    /**
     * Set the Derivative Gain constant of the PIDF controller on the SPARK MAX.
     * This uses the Set Parameter API and should be used infrequently. The
     * parameter does not presist unless burnFlash() is called.  The recommended
     * method to configure this parameter is use to SPARK MAX GUI to tune and
     * save parameters.
     *
     * @param gain   The derivative gain value, must be positive
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return CANError Set to REV_OK if successful
     */
    public CANError setD(double gain, int slotID) {
        return controller.setD(gain, slotID);
    }

    /**
     * Set the Derivative Filter constant of the PIDF controller on the SPARK MAX.
     * This uses the Set Parameter API and should be used infrequently. The
     * parameter does not presist unless burnFlash() is called.
     *
     * @param gain The derivative filter value, must be a positive number between 0 and 1
     * @return CANError Set to REV_OK if successful
     */
    public CANError setDFilter(double gain) {
        return controller.setDFilter(gain);
    }

    /**
     * Set the Derivative Filter constant of the PIDF controller on the SPARK MAX.
     * This uses the Set Parameter API and should be used infrequently. The
     * parameter does not presist unless burnFlash() is called.
     *
     * @param gain   The derivative filter value, must be a positive number between 0 and 1
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return CANError Set to REV_OK if successful
     */
    public CANError setDFilter(double gain, int slotID) {
        return controller.setDFilter(gain, slotID);
    }

    /**
     * Set the Feed-froward Gain constant of the PIDF controller on the SPARK
     * MAX. This uses the Set Parameter API and should be used infrequently. The
     * parameter does not presist unless burnFlash() is called.  The recommended
     * method to configure this parameter is use to SPARK MAX GUI to tune and
     * save parameters.
     *
     * @param gain The feed-forward gain value
     * @return CANError Set to REV_OK if successful
     */
    public CANError setFF(double gain) {
        return controller.setFF(gain);
    }

    /**
     * Set the Feed-froward Gain constant of the PIDF controller on the SPARK
     * MAX. This uses the Set Parameter API and should be used infrequently. The
     * parameter does not presist unless burnFlash() is called.  The recommended
     * method to configure this parameter is use to SPARK MAX GUI to tune and
     * save parameters.
     *
     * @param gain   The feed-forward gain value
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return CANError Set to REV_OK if successful
     */
    public CANError setFF(double gain, int slotID) {
        return controller.setFF(gain, slotID);
    }

    /**
     * Set the IZone range of the PIDF controller on the SPARK MAX. This value
     * specifies the range the |error| must be within for the integral constant
     * to take effect.
     * <p>
     * This uses the Set Parameter API and should be used infrequently.
     * The parameter does not presist unless burnFlash() is called.
     * The recommended method to configure this parameter is to use the
     * SPARK MAX GUI to tune and save parameters.
     *
     * @param IZone The IZone value, must be positive. Set to 0 to disable
     * @return CANError Set to REV_OK if successful
     */
    public CANError setIZone(double IZone) {
        return controller.setIZone(IZone);
    }

    /**
     * Set the IZone range of the PIDF controller on the SPARK MAX. This value
     * specifies the range the |error| must be within for the integral constant
     * to take effect.
     * <p>
     * This uses the Set Parameter API and should be used infrequently.
     * The parameter does not presist unless burnFlash() is called.
     * The recommended method to configure this parameter is to use the
     * SPARK MAX GUI to tune and save parameters.
     *
     * @param IZone  The IZone value, must be positive. Set to 0 to disable
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return CANError Set to REV_OK if successful
     */
    public CANError setIZone(double IZone, int slotID) {
        return controller.setIZone(IZone, slotID);
    }

    /**
     * Set the min amd max output for the closed loop mode.
     * <p>
     * This uses the Set Parameter API and should be used infrequently.
     * The parameter does not presist unless burnFlash() is called.
     * The recommended method to configure this parameter is to use the
     * SPARK MAX GUI to tune and save parameters.
     *
     * @param min Reverse power minimum to allow the controller to output
     * @param max Forward power maximum to allow the controller to output
     * @return CANError Set to REV_OK if successful
     */
    public CANError setOutputRange(double min, double max) {
        return controller.setOutputRange(min, max);
    }

    /**
     * Set the min amd max output for the closed loop mode.
     * <p>
     * This uses the Set Parameter API and should be used infrequently.
     * The parameter does not presist unless burnFlash() is called.
     * The recommended method to configure this parameter is to use the
     * SPARK MAX GUI to tune and save parameters.
     *
     * @param min    Reverse power minimum to allow the controller to output
     * @param max    Forward power maximum to allow the controller to output
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return CANError Set to REV_OK if successful
     */
    public CANError setOutputRange(double min, double max, int slotID) {
        return controller.setOutputRange(min, max, slotID);
    }

    /**
     * Get the Proportional Gain constant of the PIDF controller on the SPARK
     * MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @return double P Gain value
     */
    public double getP() {
        return controller.getP();
    }

    /**
     * Get the Proportional Gain constant of the PIDF controller on the SPARK
     * MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return double P Gain value
     */
    public double getP(int slotID) {
        return controller.getP(slotID);
    }

    /**
     * Get the Integral Gain constant of the PIDF controller on the SPARK MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @return double I Gain value
     */
    public double getI() {
        return controller.getI();
    }

    /**
     * Get the Integral Gain constant of the PIDF controller on the SPARK MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return double I Gain value
     */
    public double getI(int slotID) {
        return controller.getI(slotID);
    }

    /**
     * Get the Derivative Gain constant of the PIDF controller on the SPARK MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @return double D Gain value
     */
    public double getD() {
        return controller.getD();
    }

    /**
     * Get the Derivative Gain constant of the PIDF controller on the SPARK MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return double D Gain value
     */
    public double getD(int slotID) {
        return controller.getD(slotID);
    }

    /**
     * Get the Derivative Filter constant of the PIDF controller on the SPARK MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return double D Filter value
     */
    public double getDFilter(int slotID) {
        return controller.getDFilter(slotID);
    }

    /**
     * Get the Feed-forward Gain constant of the PIDF controller on the SPARK
     * MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @return double F Gain value
     */
    public double getFF() {
        return controller.getFF();
    }

    /**
     * Get the Feed-forward Gain constant of the PIDF controller on the SPARK
     * MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return double F Gain value
     */
    public double getFF(int slotID) {
        return controller.getFF(slotID);
    }

    /**
     * Get the IZone constant of the PIDF controller on the SPARK MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @return double IZone value
     */
    public double getIZone() {
        return controller.getIZone();
    }

    /**
     * Get the IZone constant of the PIDF controller on the SPARK MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return double IZone value
     */
    public double getIZone(int slotID) {
        return controller.getIZone(slotID);
    }

    /**
     * Get the derivative filter constant of the PIDF controller on the SPARK
     * MAX.
     *
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the gain schedule slot, the value is a number
     * between 0 and 3. Each slot has its own set of gain values and
     * can be changed in each control frame using SetReference().
     *
     * @return double D Filter
     *
     */
    // public double getDFilter(int slotID = 0);

    /**
     * Get the min output of the PIDF controller on the SPARK MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @return double min value
     */
    public double getOutputMin() {
        return controller.getOutputMin();
    }

    /**
     * Get the min output of the PIDF controller on the SPARK MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return double min value
     */
    public double getOutputMin(int slotID) {
        return controller.getOutputMin(slotID);
    }

    /**
     * Get the max output of the PIDF controller on the SPARK MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @return double max value
     */
    public double getOutputMax() {
        return controller.getOutputMax();
    }

    /**
     * Get the max output of the PIDF controller on the SPARK MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return double max value
     */
    public double getOutputMax(int slotID) {
        return controller.getOutputMax(slotID);
    }

    /**
     * Configure the maximum velocity of the SmartMotion mode. This is the
     * velocity that is reached in the middle of the profile and is what
     * the motor should spend most of its time at
     *
     * @param maxVel The maxmimum cruise velocity for the motion profile
     *               in RPM
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return CANError Set to kOK if successful
     */
    public CANError setSmartMotionMaxVelocity(double maxVel, int slotID) {
        return controller.setSmartMotionMaxVelocity(maxVel, slotID);
    }

    /**
     * Configure the maximum acceleration of the SmartMotion mode. This is
     * the accleration that the motor velocity will increase at until the
     * max velocity is reached
     *
     * @param maxAccel The maxmimum acceleration for the motion profile
     *                 in RPM per second
     * @param slotID   Is the gain schedule slot, the value is a number
     *                 between 0 and 3. Each slot has its own set of gain values and
     *                 can be changed in each control frame using SetReference().
     * @return CANError Set to kOK if successful
     */
    public CANError setSmartMotionMaxAccel(double maxAccel, int slotID) {
        return controller.setSmartMotionMaxAccel(maxAccel, slotID);
    }

    /**
     * Configure the mimimum velocity of the SmartMotion mode. Any
     * requested velocities below this value will be set to 0.
     *
     * @param minVel The minimum velocity for the motion profile in RPM
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return CANError Set to kOK if successful
     */
    public CANError setSmartMotionMinOutputVelocity(double minVel, int slotID) {
        return controller.setSmartMotionMinOutputVelocity(minVel, slotID);
    }

    /**
     * Configure the allowed closed loop error of SmartMotion mode.
     * This value is how much deviation from your setpoint is
     * tolerated and is useful in preventing oscillation around your
     * setpoint.
     *
     * @param allowedErr The allowed deviation for your setpoint vs
     *                   actual position in rotations
     * @param slotID     Is the gain schedule slot, the value is a number
     *                   between 0 and 3. Each slot has its own set of gain values and
     *                   can be changed in each control frame using SetReference().
     * @return CANError Set to kOK if successful
     */
    public CANError setSmartMotionAllowedClosedLoopError(double allowedErr, int slotID) {
        return controller.setSmartMotionAllowedClosedLoopError(allowedErr, slotID);
    }

    /**
     * Coming soon. Configure the acceleration strategy used to control
     * acceleration on the motor. The current strategy is trapezoidal
     * motion profiling.
     *
     * @param accelStrategy The acceleration strategy to use for the
     *                      automatically generated motion profile
     * @param slotID        Is the gain schedule slot, the value is a number
     *                      between 0 and 3. Each slot has its own set of gain values and
     *                      can be changed in each control frame using SetReference().
     * @return CANError Set to kOK if successful
     */
    public CANError setSmartMotionAccelStrategy(CANPIDController.AccelStrategy accelStrategy, int slotID) {
        return controller.setSmartMotionAccelStrategy(accelStrategy, slotID);
    }

    /**
     * Get the maximum velocity of the SmartMotion mode. This is the
     * velocity that is reached in the middle of the profile and is
     * what the motor should spend most of its time at
     *
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return The maxmimum cruise velocity for the motion profile in
     * RPM
     */
    public double getSmartMotionMaxVelocity(int slotID) {
        return controller.getSmartMotionMaxVelocity(slotID);
    }

    /**
     * Get the maximum acceleration of the SmartMotion mode. This is
     * the accleration that the motor velocity will increase at until
     * the max velocity is reached
     *
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return The maxmimum acceleration for the motion profile in
     * RPM per second
     */
    public double getSmartMotionMaxAccel(int slotID) {
        return controller.getSmartMotionMaxAccel(slotID);
    }

    /**
     * Get the mimimum velocity of the SmartMotion mode. Any requested
     * velocities below this value will be set to 0.
     *
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return The minimum velocity for the motion profile in RPM
     */
    public double getSmartMotionMinOutputVelocity(int slotID) {
        return controller.getSmartMotionMinOutputVelocity(slotID);
    }

    /**
     * Get the allowed closed loop error of SmartMotion mode. This
     * value is how much deviation from your setpoint is tolerated
     * and is useful in preventing oscillation around your setpoint.
     *
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return The allowed deviation for your setpoint vs actual
     * position in rotations
     */
    public double getSmartMotionAllowedClosedLoopError(int slotID) {
        return controller.getSmartMotionAllowedClosedLoopError(slotID);
    }

    /**
     * Get the acceleration strategy used to control acceleration on
     * the motor.The current strategy is trapezoidal motion profiling.
     *
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return The acceleration strategy to use for the automatically
     * generated motion profile.
     */
    public CANPIDController.AccelStrategy getSmartMotionAccelStrategy(int slotID) {
        return controller.getSmartMotionAccelStrategy(slotID);
    }

    /**
     * Configure the maximum I accumulator of the PID controller.
     * This value is used to constrain the I accumulator to help
     * manage integral wind-up
     *
     * @param iMaxAccum The max value to contrain the I accumulator to
     * @param slotID    Is the gain schedule slot, the value is a number
     *                  between 0 and 3. Each slot has its own set of gain values and
     *                  can be changed in each control frame using SetReference().
     * @return CANError Set to kOK if successful
     */
    public CANError setIMaxAccum(double iMaxAccum, int slotID) {
        return controller.setIMaxAccum(iMaxAccum, slotID);
    }

    /**
     * Get the maximum I accumulator of the PID controller.
     * This value is used to constrain the I accumulator to help manage
     * integral wind-up
     *
     * @param slotID Is the gain schedule slot, the value is a number
     *               between 0 and 3. Each slot has its own set of gain values and
     *               can be changed in each control frame using SetReference().
     * @return The max value to contrain the I accumulator to
     */
    public double getIMaxAccum(int slotID) {
        return controller.getIMaxAccum(slotID);
    }

    /**
     * Set the I accumulator of the PID controller. This is useful
     * when wishing to force a reset on the I accumulator of the
     * PID controller. You can also preset values to see how it
     * will respond to certain I characteristics
     * <p>
     * To use this function, the controller must be in a closed
     * loop control mode by calling setReference()
     *
     * @param iAccum The value to set the I accumulator to
     * @return CANError Set to kOK if successful
     */
    public CANError setIAccum(double iAccum) {
        return controller.setIAccum(iAccum);
    }

    /**
     * Get the I accumulator of the PID controller. This is useful
     * when wishing to see what the I accumulator value is to help
     * with PID tuning
     *
     * @return The value of the I accumulator
     */
    public double getIAccum() {
        return controller.getIAccum();
    }

    /**
     * Get the firmware version of the SPARK MAX.
     *
     * @return uint32_t Firmware version integer. Value is represented as 4 bytes,
     * Major.Minor.Build H.Build L
     */
    public int getFirmwareVersion() {
        return super.getFirmwareVersion();
    }

    /**
     * Get the firmware version of the SPARK MAX as a string.
     *
     * @return std::string Human readable firmware version string
     */
    public String getFirmwareString() {
        return super.getFirmwareString();
    }

    /**
     * Get the unique serial number of the SPARK MAX. Not currently available.
     *
     * @return byte[] Vector of bytes representig the unique serial number
     */
    public byte[] getSerialNumber() {
        return super.getSerialNumber();
    }

    /**
     * Get the configured Device ID of the SPARK MAX.
     *
     * @return int device ID
     */
    public int getDeviceId() {
        return super.getDeviceId();
    }

    /**
     * Set the motor type connected to the SPARK MAX.
     * <p>
     * This uses the Set Parameter API and should be used infrequently. The
     * parameter does not presist unless burnFlash() is called. The recommended
     * method to configure this parameter is to use the SPARK MAX GUI to tune and
     * save parameters.
     *
     * @param type The type of motor connected to the controller. Brushless motors
     *             must be connected to their matching color and the hall sensor
     *             plugged in. Brushed motors must be connected to the Red and Black
     *             terminals only.
     * @return CANError Set to CANError::kOk if successful
     */
    public CANError setMotorType(MotorType type) {
        return super.setMotorType(type);
    }

    /**
     * Get the motor type setting for the SPARK MAX.
     * <p>
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @return MotorType Motor type setting
     */
    public MotorType getMotorType() {
        return super.getMotorType();
    }

    /**
     * Set the rate of transmission for periodic frames from the SPARK MAX
     * <p>
     * Each motor controller sends back three status frames with different data at
     * set rates. Use this function to change the default rates.
     * <p>
     * Defaults: Status0 - 10ms Status1 - 20ms Status2 - 50ms
     * <p>
     * This value is not stored in the FLASH after calling burnFlash() and is reset
     * on powerup.
     * <p>
     * Refer to the SPARK MAX reference manual on details for how and when to
     * configure this parameter.
     *
     * @param frameID  The frame ID can be one of PeriodicFrame type
     * @param periodMs The rate the controller sends the frame to the controller.
     * @return CANError Set to CANError.kOk if successful
     */
    public CANError setPeriodicFramePeriod(PeriodicFrame frameID, int periodMs) {
        return super.setPeriodicFramePeriod(frameID, periodMs);
    }

    public ParameterStatus setParameter(ConfigParameter parameterID, double value) {
        return super.setParameter(parameterID, value);
    }

    public ParameterStatus setParameter(ConfigParameter parameterID, int value) {
        return super.setParameter(parameterID, value);
    }

    public ParameterStatus setParameter(ConfigParameter parameterID, boolean value) {
        return super.setParameter(parameterID, value);
    }

    public Optional<Double> getParameterDouble(ConfigParameter parameterID) {
        return super.getParameterDouble(parameterID);
    }

    public Optional<Integer> getParameterInt(ConfigParameter parameterID) {
        return super.getParameterInt(parameterID);
    }

    public Optional<Boolean> getParameterBoolean(ConfigParameter parameterID) {
        return super.getParameterBoolean(parameterID);
    }

    public CANError setEncPosition(double value) {
        return super.setEncPosition(value);
    }

    /**
     * Restore motor controller parameters to factory default until the next
     * controller reboot
     *
     * @return CANError Set to CANError::kOk if successful
     */
    public CANError restoreFactoryDefaults() {
        return super.restoreFactoryDefaults();
    }

    /**
     * Restore motor controller parameters to factory default
     *
     * @param persist If true, burn the flash with the factory default parameters
     * @return CANError Set to CANError::kOk if successful
     */
    public CANError restoreFactoryDefaults(boolean persist) {
        return super.restoreFactoryDefaults(persist);
    }

    public ParameterStatus setParameterCore(ConfigParameter parameterID, ParameterType type, int value) {
        return super.setParameterCore(parameterID, type, value);
    }

    public Optional<Integer> getParameterCore(ConfigParameter parameterID, ParameterType expectedType) {
        return super.getParameterCore(parameterID, expectedType);
    }

    public ParameterType getParameterType(ConfigParameter parameterID) {
        return super.getParameterType(parameterID);
    }
}
