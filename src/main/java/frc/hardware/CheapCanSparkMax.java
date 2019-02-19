package frc.hardware;

import com.revrobotics.CANSparkMax;

import static frc.utils.Constants.EPSILON_SMALL_DOUBLE;

/**
 * Small wrapper around {@link CANSparkMax} to reduce CAN usage and overhead. Only flushes output when the
 * desired parameters are different than the current parameters.
 */
public class CheapCanSparkMax extends CANSparkMax {
    private double lastSpeed = Double.NaN;

    /**
     * Constructor.
     *
     * @param deviceId  the device ID of the Spark MAX, as addressed on the CAN network.
     * @param motorType the motor type being controlled by the motor controller, either {@link MotorType#kBrushed} or
     *                  {@link MotorType#kBrushless}.
     */
    public CheapCanSparkMax(int deviceId, MotorType motorType) {
        super(deviceId, motorType);
    }

    @Override
    public void set(double speed) {
        if (Math.abs(speed - lastSpeed) > EPSILON_SMALL_DOUBLE) {
            super.set(speed);
            lastSpeed = speed;
        }
    }
}
