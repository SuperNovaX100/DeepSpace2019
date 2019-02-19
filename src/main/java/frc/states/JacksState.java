package frc.states;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.subsystem.Jack0409;
import frc.utils.DriveSignal;

public class JacksState {
    public SingleJackInputData frontJackInput = new SingleJackInputData();
    public SingleJackInputData leftJackInput = new SingleJackInputData();
    public SingleJackInputData rightJackInput = new SingleJackInputData();
    public SingleJackOutputData frontJackOutput = new SingleJackOutputData();
    public SingleJackOutputData leftJackOutput = new SingleJackOutputData();
    public SingleJackOutputData rightJackOutput = new SingleJackOutputData();
    public GeneralInput generalInput = new GeneralInput();
    public GeneralOutput generalOutput = new GeneralOutput();
    public double leftWheelDemand;
    public double rightWheelDemand;

    public static class GeneralInput {
        public double pitch;
        public double roll;
        public boolean frontIrDetectsGround;
        public boolean rearIrDetectsGround;
        public double finishTimestamp;
        public Jack0409.JackSystem state = Jack0409.JackSystem.OPEN_LOOP;
        public Jack0409.GainsState lastConfiguredGainState = null;
        public Jack0409.JackState habLevelToClimbTo = Jack0409.JackState.RETRACT;
    }

    public static class GeneralOutput {
        public boolean sendSignalToDriveBase = false;
        public DriveSignal driveBaseDriveSignal = DriveSignal.NEUTRAL;
        public Jack0409.GainsState desiredGainState = null;
    }

    public static class SingleJackInputData {
        public double encoder;
        public double currentDraw;
        public boolean resetEncoder;
        public boolean hasZeroed;
    }

    public static class SingleJackOutputData {
        public ControlMode controlMode = ControlMode.PercentOutput;
        public double demand = 0.0;
        public double feedForward = 0.0;
    }
}
