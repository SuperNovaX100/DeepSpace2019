package frc.utils;

import org.junit.Assert;
import org.junit.Test;

import static frc.utils.DpadHelper.*;

public class DpadHelperTest {
    @Test
    public void rightLastState() {
        final DpadHelper.LastDpadState lastDpadState = DpadHelper.LastDpadState.RIGHT;
        Assert.assertEquals(LastDpadState.UP, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_UP));
        Assert.assertEquals(LastDpadState.DOWN, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_DOWN));
        Assert.assertEquals(LastDpadState.LEFT, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_LEFT));
        Assert.assertEquals(LastDpadState.RIGHT, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_RIGHT));
        Assert.assertEquals(LastDpadState.RIGHT, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_UPPER_RIGHT));
        Assert.assertEquals(LastDpadState.RIGHT, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_LOWER_RIGHT));
        Assert.assertEquals(LastDpadState.NONE, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_UPPER_LEFT));
        Assert.assertEquals(LastDpadState.NONE, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_LOWER_LEFT));
    }

    @Test
    public void leftLastState() {
        final DpadHelper.LastDpadState lastDpadState = LastDpadState.LEFT;
        Assert.assertEquals(LastDpadState.UP, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_UP));
        Assert.assertEquals(LastDpadState.DOWN, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_DOWN));
        Assert.assertEquals(LastDpadState.LEFT, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_LEFT));
        Assert.assertEquals(LastDpadState.RIGHT, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_RIGHT));
        Assert.assertEquals(LastDpadState.NONE, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_UPPER_RIGHT));
        Assert.assertEquals(LastDpadState.NONE, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_LOWER_RIGHT));
        Assert.assertEquals(LastDpadState.LEFT, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_UPPER_LEFT));
        Assert.assertEquals(LastDpadState.LEFT, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_LOWER_LEFT));
    }

    @Test
    public void upLastState() {
        final DpadHelper.LastDpadState lastDpadState = LastDpadState.UP;
        Assert.assertEquals(LastDpadState.UP, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_UP));
        Assert.assertEquals(LastDpadState.DOWN, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_DOWN));
        Assert.assertEquals(LastDpadState.LEFT, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_LEFT));
        Assert.assertEquals(LastDpadState.RIGHT, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_RIGHT));
        Assert.assertEquals(LastDpadState.UP, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_UPPER_RIGHT));
        Assert.assertEquals(LastDpadState.NONE, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_LOWER_RIGHT));
        Assert.assertEquals(LastDpadState.UP, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_UPPER_LEFT));
        Assert.assertEquals(LastDpadState.NONE, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_LOWER_LEFT));
    }

    @Test
    public void downLastState() {
        final DpadHelper.LastDpadState lastDpadState = LastDpadState.DOWN;
        Assert.assertEquals(LastDpadState.UP, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_UP));
        Assert.assertEquals(LastDpadState.DOWN, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_DOWN));
        Assert.assertEquals(LastDpadState.LEFT, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_LEFT));
        Assert.assertEquals(LastDpadState.RIGHT, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_RIGHT));
        Assert.assertEquals(LastDpadState.NONE, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_UPPER_RIGHT));
        Assert.assertEquals(LastDpadState.DOWN, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_LOWER_RIGHT));
        Assert.assertEquals(LastDpadState.NONE, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_UPPER_LEFT));
        Assert.assertEquals(LastDpadState.DOWN, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_LOWER_LEFT));
    }

    @Test
    public void noneLastState() {
        final DpadHelper.LastDpadState lastDpadState = LastDpadState.NONE;
        Assert.assertEquals(LastDpadState.UP, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_UP));
        Assert.assertEquals(LastDpadState.DOWN, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_DOWN));
        Assert.assertEquals(LastDpadState.LEFT, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_LEFT));
        Assert.assertEquals(LastDpadState.RIGHT, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_RIGHT));
        Assert.assertEquals(LastDpadState.NONE, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_UPPER_RIGHT));
        Assert.assertEquals(LastDpadState.NONE, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_LOWER_RIGHT));
        Assert.assertEquals(LastDpadState.NONE, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_UPPER_LEFT));
        Assert.assertEquals(LastDpadState.NONE, DpadHelper.lastDpadUpdate(lastDpadState, POV_DPAD_LOWER_LEFT));
    }
}
