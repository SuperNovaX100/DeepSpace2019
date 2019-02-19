package frc.statemachines;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.states.JacksState;
import frc.subsystem.Jack0409;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import static frc.statemachines.JacksStateMachine.MAX_AMP_DRAW_ZEROING;
import static frc.utils.Constants.EPSILON_SMALL_DOUBLE;

public class JackStateMachineTest {
    private JacksStateMachine jacksStateMachine;
    private JacksState jacksState;

    @Before
    public void setup() {
        jacksStateMachine = new JacksStateMachine();
        jacksState = new JacksState();
    }

    @Test
    public void stop() {
        jacksState.generalInput.state = Jack0409.JackSystem.STOP;
        jacksState = jacksStateMachine.update(jacksState, 0);
        Assert.assertEquals(jacksState.leftWheelDemand, 0, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(jacksState.rightWheelDemand, 0, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(jacksState.frontJackOutput.demand, 0, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(jacksState.leftJackOutput.demand, 0, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(jacksState.rightJackOutput.demand, 0, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(jacksState.frontJackOutput.feedForward, 0, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(jacksState.leftJackOutput.feedForward, 0, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(jacksState.rightJackOutput.feedForward, 0, EPSILON_SMALL_DOUBLE);
        Assert.assertEquals(jacksState.frontJackOutput.controlMode, ControlMode.PercentOutput);
        Assert.assertEquals(jacksState.leftJackOutput.controlMode, ControlMode.PercentOutput);
        Assert.assertEquals(jacksState.rightJackOutput.controlMode, ControlMode.PercentOutput);
    }

    @Test
    public void zeroing() {
        jacksState.generalInput.state = Jack0409.JackSystem.ZEROING;
        jacksState.frontJackInput.currentDraw = 0;
        jacksState.leftJackInput.currentDraw = 0;
        jacksState.rightJackInput.currentDraw = 0;
        jacksState = jacksStateMachine.update(jacksState, 0);
        final int iterations = 300;
        final int frontZeroIter = iterations / 4;
        final int leftZeroIter = 2 * iterations / 4;
        final int rightZeroIter = 3 * iterations / 4;
        for (int i = 0; i < iterations; i++) {
            if (i == frontZeroIter) {
                jacksState.frontJackInput.currentDraw = MAX_AMP_DRAW_ZEROING * 2;
            } else {
                jacksState.frontJackInput.currentDraw = 0;
            }

            if (i == leftZeroIter) {
                jacksState.leftJackInput.currentDraw = MAX_AMP_DRAW_ZEROING * 2;
            } else {
                jacksState.leftJackInput.currentDraw = 0;
            }
            if (i == rightZeroIter) {
                jacksState.rightJackInput.currentDraw = MAX_AMP_DRAW_ZEROING * 2;
            } else {
                jacksState.rightJackInput.currentDraw = 0;
            }

            jacksState = jacksStateMachine.update(jacksState, i);

            if (i >= frontZeroIter) {
                Assert.assertEquals(0, jacksState.frontJackOutput.demand, EPSILON_SMALL_DOUBLE);
                Assert.assertEquals(ControlMode.PercentOutput, jacksState.frontJackOutput.controlMode);
            }
            if (i >= leftZeroIter) {
                Assert.assertEquals(0, jacksState.leftJackOutput.demand, EPSILON_SMALL_DOUBLE);
                Assert.assertEquals(ControlMode.PercentOutput, jacksState.leftJackOutput.controlMode);
            }
            if (i >= rightZeroIter) {
                Assert.assertEquals(0, jacksState.rightJackOutput.demand, EPSILON_SMALL_DOUBLE);
                Assert.assertEquals(ControlMode.PercentOutput, jacksState.rightJackOutput.controlMode);
            }
        }
        Assert.assertEquals(Jack0409.JackSystem.STOP, jacksState.generalInput.state);
    }

    // TODO finish writing more complete test cases, should include but not be limited to
    //  ensuring that with things like going up, if encoders go past a point in a certain direction, they are counted
    //  by checkEncoders
    // TODO evaluate the reliability of recent zeroing (within n seconds, or perhaps if all are within k encoder units
    //  of zero) on climbing, in order to shave time off of the climb.
}
