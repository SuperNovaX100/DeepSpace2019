package frc.statemachines;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.states.HatchState;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import static frc.statemachines.HatchStateMachine.ZEROING_SPEED;
import static frc.utils.Constants.*;

public class HatchStateMachineTest {
    private HatchStateMachine hatchStateMachine;
    private HatchState hatchState;
    private HatchState outputState;

    @Before
    public void setup() {
        hatchStateMachine = new HatchStateMachine();
        hatchState = new HatchState();
        outputState = new HatchState();
    }

    @Test
    public void limitSwitchHit() {
        hatchState.limitHit = true;
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertTrue(outputState.limitHit);
        Assert.assertEquals(HatchStateMachine.PEAK_REVERSE_OUTPUT_LIMIT_PRESSED, outputState.peakOutputReverse, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(HatchStateMachine.PEAK_FORWARD_OUTPUT_LIMIT_PRESSED, outputState.peakOutputForward, EPSILON_SMALL_DOUBLE);
    }

    @Test
    public void resetEncoderOnClick() {
        hatchState.limitHit = true;
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertTrue(outputState.resetSensor);
    }

    @Test
    public void noResetEncoderOnLimitHeld() {
        hatchState.limitHit = true;
        outputState = hatchStateMachine.update(hatchState);
        hatchState.limitHit = true;
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertFalse(outputState.resetSensor);
    }

    @Test
    public void noResetEncoderOnLimitNotTouched() {
        hatchState.limitHit = true;
        outputState = hatchStateMachine.update(hatchState);
        hatchState.limitHit = false;
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertFalse(outputState.resetSensor);
    }

    @Test
    public void limitSwitchNotHit() {
        hatchState.limitHit = false;
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertFalse(outputState.limitHit);
        Assert.assertEquals(HatchStateMachine.PEAK_REVERSE_OUTPUT_STANDARD, outputState.peakOutputReverse, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(HatchStateMachine.PEAK_FORWARD_OUTPUT_STANDARD, outputState.peakOutputForward, EPSILON_SMALL_DOUBLE);
    }

    @Test
    public void setOpenLoopHoldPositionNeutralPower() {
        final double neutralPower = 0.0;
        final double nonNeutralPower = 1.0;

        final int initialEncoderValueOne = -1000;
        final int initialEncoderValueTwo = 1000;
        final int encoderDelta = 100;
        for (int encoderValue = initialEncoderValueOne; encoderValue < 1000; encoderValue += encoderDelta) {
            hatchState.encoder = encoderValue;
            setOpenLoop(neutralPower);
            outputState = hatchStateMachine.update(hatchState);
            Assert.assertEquals(ControlMode.MotionMagic, outputState.controlMode);
            Assert.assertEquals(initialEncoderValueOne, outputState.demand, EPSILON_SMALL_DOUBLE);
        }
        setOpenLoop(nonNeutralPower);
        outputState = hatchStateMachine.update(hatchState);
        for (int encoderValue = initialEncoderValueTwo; encoderValue > -1000; encoderValue -= encoderDelta) {
            hatchState.encoder = encoderValue;
            setOpenLoop(neutralPower);
            outputState = hatchStateMachine.update(hatchState);
            Assert.assertEquals(ControlMode.MotionMagic, outputState.controlMode);
            Assert.assertEquals(initialEncoderValueTwo, outputState.demand, EPSILON_SMALL_DOUBLE);
        }
    }

    @Test
    public void setOpenLoopFullPowerForward() {
        final double power = 1.0;
        setOpenLoop(power);
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertEquals(power, outputState.demand, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(ControlMode.PercentOutput, outputState.controlMode);
    }

    @Test
    public void setOpenLoopFullPowerReverse() {
        final double power = -1.0;
        setOpenLoop(power);
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertEquals(power, outputState.demand, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(ControlMode.PercentOutput, outputState.controlMode);
    }

    @Test
    public void setOpenLoopPartialPowerReverse() {
        final double power = -0.3;
        setOpenLoop(power);
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertEquals(power, outputState.demand, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(ControlMode.PercentOutput, outputState.controlMode);
    }

    @Test
    public void setOpenLoopPartialPowerForward() {
        final double power = 0.3;
        setOpenLoop(power);
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertEquals(power, outputState.demand, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(ControlMode.PercentOutput, outputState.controlMode);
    }

    @Test
    public void setOpenLoopForwardLimitSwitchHit() {
        final double power = 1.0;
        setOpenLoop(power);
        hatchState.limitHit = true;
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertEquals(power, outputState.demand, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(ControlMode.PercentOutput, outputState.controlMode);
    }

    @Test
    public void setOpenLoopReverseLimitSwitchHit() {
        final double power = -1.0;
        setOpenLoop(power);
        hatchState.limitHit = true;
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertEquals(power, outputState.demand, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(ControlMode.PercentOutput, outputState.controlMode);
    }

    @Test
    public void setHatchPlaceHasZeroed() {
        zeroEndWithLimitNotHit();
        setHatchPlace();
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertEquals(HATCH_PLACE_ENCODER_POSITION, outputState.demand, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(ControlMode.MotionMagic, outputState.controlMode);
    }

    @Test
    public void setHatchPlaceHasNotZeroed() {
        setHatchPlace();
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertNotEquals(HATCH_PLACE_ENCODER_POSITION, outputState.demand, EPSILON_SMALL_DOUBLE);
        Assert.assertNotEquals(ControlMode.MotionMagic, outputState.controlMode);
        ensureZeroing();
    }

    @Test
    public void setHatchPullHasZeroed() {
        zeroEndWithLimitNotHit();
        setHatchPull();
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertEquals(HATCH_PULL_ENCODER_POSITION, outputState.demand, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(ControlMode.MotionMagic, outputState.controlMode);
    }

    @Test
    public void setHatchPullHasNotZeroed() {
        setHatchPull();
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertNotEquals(HATCH_PULL_ENCODER_POSITION, outputState.demand, EPSILON_SMALL_DOUBLE);
        Assert.assertNotEquals(ControlMode.MotionMagic, outputState.controlMode);
        ensureZeroing();
    }

    @Test
    public void hasZeroedInitialLimitHit() {
        hatchState.limitHit = true;
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertTrue(outputState.hasZeroed);
    }

    @Test
    public void hasZeroedMultipleLimitHitsAndRemovals() {
        zeroEndWithLimitNotHit();
        zeroEndWithLimitNotHit();
        Assert.assertTrue(outputState.hasZeroed);
    }

    @Test
    public void hasZeroedNoLimitEverHit() {
        hatchState.limitHit = false;
        outputState = hatchStateMachine.update(hatchState);
        Assert.assertFalse(outputState.hasZeroed);
    }

    private void ensureZeroing() {
        Assert.assertEquals(ZEROING_SPEED, outputState.demand, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(ControlMode.PercentOutput, outputState.controlMode);
    }

    private void zeroEndWithLimitNotHit() {
        hatchState.limitHit = true;
        outputState = hatchStateMachine.update(hatchState);
        hatchState.limitHit = false;
        outputState = hatchStateMachine.update(hatchState);
    }

    private void setOpenLoop(double joystickPower) {
        hatchState.desiredControlState = HatchState.ControlState.OPEN_LOOP;
        hatchState.desiredDemand = joystickPower;
    }

    private void setHatchPlace() {
        hatchState.desiredControlState = HatchState.ControlState.HATCH_PLACE_POSITION;
    }

    private void setHatchPull() {
        hatchState.desiredControlState = HatchState.ControlState.HATCH_PULL_POSITION;
    }
}
