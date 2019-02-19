package frc.utils;

public final class DpadHelper {
    static final int POV_DPAD_UP = 0;
    static final int POV_DPAD_UPPER_RIGHT = 45;
    static final int POV_DPAD_LOWER_RIGHT = 135;
    static final int POV_DPAD_RIGHT = 90;
    static final int POV_DPAD_DOWN = 180;
    static final int POV_DPAD_LEFT = 270;
    static final int POV_DPAD_UPPER_LEFT = 315;
    static final int POV_DPAD_LOWER_LEFT = 225;

    private DpadHelper(){

    }

    public static LastDpadState lastDpadUpdate(LastDpadState lastDpadState, int pov) {
        if (pov == -1) {
            return LastDpadState.NONE;
        } else if (pov == POV_DPAD_DOWN) {
            return LastDpadState.DOWN;
        } else if (pov == POV_DPAD_RIGHT) {
            return LastDpadState.RIGHT;
        } else if (pov == POV_DPAD_LEFT) {
            return LastDpadState.LEFT;
        } else if (pov == POV_DPAD_UP) {
            return LastDpadState.UP;
        } else if (pov == POV_DPAD_UPPER_RIGHT) {
            return lastDpadState == LastDpadState.RIGHT ? LastDpadState.RIGHT : (lastDpadState == LastDpadState.UP ? LastDpadState.UP : LastDpadState.NONE);
        } else if (pov == POV_DPAD_LOWER_RIGHT) {
            return lastDpadState == LastDpadState.RIGHT ? LastDpadState.RIGHT : (lastDpadState == LastDpadState.DOWN ? LastDpadState.DOWN : LastDpadState.NONE);
        } else if (pov == POV_DPAD_UPPER_LEFT) {
            return lastDpadState == LastDpadState.LEFT ? LastDpadState.LEFT : (lastDpadState == LastDpadState.UP ? LastDpadState.UP : LastDpadState.NONE);
        } else if (pov == POV_DPAD_LOWER_LEFT) {
            return lastDpadState == LastDpadState.LEFT ? LastDpadState.LEFT : (lastDpadState == LastDpadState.DOWN ? LastDpadState.DOWN : LastDpadState.NONE);
        } else {
            return lastDpadState;
        }
    }

    /**
     * The last recorded value of the DPAD.
     *
     * <p>
     * Since the DPAD will quite often return a 45 degrees bisection between any of the four clickable directions, when
     * holding one side, jittering in and out of that value can happen, leading to jittering on the motors. By storing
     * the previously held state as one of the four valid directions, when the DPAD returns one of the 45 degree
     * bisections, the last state can be used to make an educated guess as to what is actually being held.
     */
    public enum LastDpadState {
        UP,
        DOWN,
        LEFT,
        RIGHT,
        NONE
    }

}
