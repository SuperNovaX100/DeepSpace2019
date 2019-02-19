package frc.utils;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

/**
 * Adds a reset function to the {@link CANEncoder} class.
 * TODO see if {@link com.revrobotics.CANSparkMaxLowLevel#setEncPosition(double)} is a suitable alternative.
 */
public class ResettableCanEncoder extends CANEncoder {
    private double offset;

    /**
     * Constructs a CANPIDController.
     *
     * @param device The Spark Max to which the encoder is attached.
     */
    public ResettableCanEncoder(CANSparkMax device) {
        super(device);
    }

    public double getPosition() {
        return super.getPosition() + offset;
    }

    public void resetEncoder() {
        resetEncoder(0);
    }

    public void resetEncoder(double desiredCurrentValue) {
        offset = -super.getPosition() + desiredCurrentValue;
    }
}
