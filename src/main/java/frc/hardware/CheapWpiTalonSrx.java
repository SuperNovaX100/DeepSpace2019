package frc.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Small wrapper around {@link WPI_TalonSRX} to reduce CAN usage and overhead. Only flushes output when the
 * desired parameters are different than the current parameters.
 */
public class CheapWpiTalonSrx extends WPI_TalonSRX {
    // The last modes and values that were actually output
    private ControlMode lastControlMode = null;
    private double lastDemand = Double.NaN;
    private DemandType lastDemandType = null;
    private double lastDemand1 = Double.NaN;

    /**
     * Constructor.
     *
     * @param deviceNumber the device number of the Talon SRX, as given by Phoenix Tuner in the case of the CAN bus.
     */
    public CheapWpiTalonSrx(int deviceNumber) {
        // Since WPI_TalonSRX is the parent (super) class, calls that constructor
        // i.e. "new CheapWpiTalonSrx(5)" is equivalent in functionality to "new WPI_TalonSRX(5)"
        super(deviceNumber);
    }

    /**
     * Sets the Talon SRX to the desired {@link ControlMode} and demand, writes output if the desired values are
     * different than the currently applied ones. Calls to {@link WPI_TalonSRX#set(ControlMode, double)} when an updated
     * value is desired.
     *
     * @param controlMode the {@link ControlMode} to set the Talon SRX to.
     * @param demand      the demand to set the Talon SRX to, e.g. 0.5 for 50% of output in
     *                    {@link ControlMode#PercentOutput}.
     */
    @Override
    public void set(ControlMode controlMode, double demand) {
        if (demand != lastDemand || controlMode != lastControlMode) {
            super.set(controlMode, demand);
            lastControlMode = controlMode;
            lastDemand = demand;
        }
    }

    /**
     * Sets the Talon SRX to the desired {@link ControlMode} and demand, writes output if the desired values are
     * different than the currently applied ones. Calls to {@link WPI_TalonSRX#set(ControlMode, double)} when an updated
     * value is desired.
     *
     * @param controlMode the {@link ControlMode} to set the Talon SRX to.
     * @param demand0     the demand to set the Talon SRX to, e.g. 0.5 for 50% of output in
     *                    {@link ControlMode#PercentOutput}.
     * @param demandType  the {@link DemandType} to set the Talon SRX to follow.
     * @param demand1     the demand to set for the {@link DemandType} parameter.
     *                    {@link ControlMode#PercentOutput}.
     */
    @Override
    public void set(ControlMode controlMode, double demand0, DemandType demandType, double demand1) {
        if (demand0 != lastDemand || demand1 != lastDemand1 || controlMode != lastControlMode || demandType != lastDemandType) {
            super.set(controlMode, demand0, demandType, demand1);
            lastDemand = demand0;
            lastDemand1 = demand1;
            lastControlMode = controlMode;
            lastDemandType = demandType;
        }
    }
}
