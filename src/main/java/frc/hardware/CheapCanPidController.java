package frc.hardware;

import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

/**
 * Small wrapper around {@link CANPIDController} to reduce CAN usage and overhead. Only flushes output when the
 * desired parameters are different than the current parameters.
 */
public class CheapCanPidController extends CANPIDController {
    private static final int SLOTS = 4;
    private final double[] lastValue = new double[SLOTS];
    private final ControlType[] lastControlType = new ControlType[SLOTS];
    private final double[] lastArbFeedForward = new double[SLOTS];
    private final double[] lastPValues = new double[SLOTS];

    /**
     * Constructor.
     *
     * @param device the device to acquire the PID controller for.
     */
    public CheapCanPidController(CANSparkMax device) {
        super(device);
        for (int i = 0; i < SLOTS; i++) {
            lastValue[i] = Double.NaN;
            lastArbFeedForward[i] = Double.NaN;
            lastControlType[i] = null;
            lastPValues[i] = Double.NaN;
        }
    }

    /**
     * Sets the desired reference point for the PID controller in a given slot. If any of the values are different than
     * the previous time that values were written, they will be written to the motor controller again. If, however,
     * all of the values are the same, nothing will be output to the motors.
     *
     * @param value       the set point for the pid controller.
     * @param controlType the {@link ControlType} to use as the control mode for the PID controller.
     * @return an {@link CANError} reporting the success or failure of the operation. A {@link CANError#kOK} implies
     * that the command was successfully carried out with no errors.
     */
    public CANError setReference(double value, ControlType controlType) {
        return setReference(value, controlType, 0);
    }

    /**
     * Sets the desired reference point for the PID controller in a given slot. If any of the values are different than
     * the previous time that values were written, they will be written to the motor controller again. If, however,
     * all of the values are the same, nothing will be output to the motors.
     *
     * @param value       the set point for the pid controller.
     * @param controlType the {@link ControlType} to use as the control mode for the PID controller.
     * @param pidSlot     the pid slot that the values should be set for. Must be in range [0, 3].
     * @return an {@link CANError} reporting the success or failure of the operation. A {@link CANError#kOK} implies
     * that the command was successfully carried out with no errors.
     */
    public CANError setReference(double value, ControlType controlType, int pidSlot) {
        return setReference(value, controlType, pidSlot, 0.0);
    }

    /**
     * Sets the desired reference point for the PID controller in a given slot. If any of the values are different than
     * the previous time that values were written, they will be written to the motor controller again. If, however,
     * all of the values are the same, nothing will be output to the motors.
     *
     * @param value          the set point for the pid controller.
     * @param controlType    the {@link ControlType} to use as the control mode for the PID controller.
     * @param pidSlot        the pid slot that the values should be set for. Must be in range [0, 3].
     * @param arbFeedForward an arbitrary feed forward output in percent output.
     * @return an {@link CANError} reporting the success or failure of the operation. A {@link CANError#kOK} implies
     * that the command was successfully carried out with no errors.
     */
    public CANError setReference(double value, ControlType controlType, int pidSlot, double arbFeedForward) {
        if (pidSlot < 0 || pidSlot >= SLOTS) {
            return CANError.kError;
        }

        if (value != lastValue[pidSlot] || controlType != lastControlType[pidSlot] || arbFeedForward != lastArbFeedForward[pidSlot]) {
            lastValue[pidSlot] = value;
            lastControlType[pidSlot] = controlType;
            lastArbFeedForward[pidSlot] = pidSlot;
            return super.setReference(value, controlType, pidSlot, arbFeedForward);
        }
        return CANError.kOK;
    }

    public CANError setP(double gain) {
        return setP(gain, 0);
    }

    public CANError setP(double gain, int slotId) {
        if (slotId < 0 || slotId >= SLOTS) {
            return CANError.kError;
        }

        if (lastPValues[slotId] != gain) {
            lastPValues[slotId] = gain;
            return super.setP(gain, slotId);
        }

        return CANError.kOK;
    }
}
