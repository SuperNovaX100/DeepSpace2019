package frc.loops;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

import static frc.utils.ShuffleboardConstants.LOOPER_SHUFFLEBOARD;

/**
 * Takes groups of {@link Loop}s and executes their functions periodically via an {@link Notifier}..
 */
public class Looper implements LooperInterface {
    /**
     * The period that all loops should run at when registered in a {@link Looper}.
     */
    public static final double PERIOD = 0.01;
    private final Notifier notifier;
    private final List<Loop> loops;
    private final Object runningLock = new Object();
    private boolean running;
    private double timestamp = 0.0;
    private double dt = 0.0;

    /**
     * Constructor.
     */
    public Looper() {
        Runnable runnable = new Runnable() {
            @Override
            public void run() {
                synchronized (runningLock) {
                    if (running) {
                        double now = Timer.getFPGATimestamp();
                        for (Loop loop : loops) {
                            loop.onLoop(now);
                        }
                        dt = now - timestamp;
                        timestamp = now;
                    }
                }
            }
        };
        notifier = new Notifier(runnable);
        running = false;
        loops = new ArrayList<>();
    }

    /**
     * Adds a loop to the list of loops to use and call periodically.
     *
     * @param loop the {@link Loop} to add to the looper.
     */
    public synchronized void registerLoop(Loop loop) {
        synchronized (runningLock) {
            loops.add(loop);
        }
    }

    /**
     * Starts running all loops by calling their {@link Loop#onStart(double)} function.
     */
    public synchronized void start() {
        if (!running) {
            synchronized (runningLock) {
                timestamp = Timer.getFPGATimestamp();
                for (Loop loop : loops) {
                    loop.onStart(timestamp);
                }
                running = true;
            }
            notifier.startPeriodic(PERIOD);
        }
    }

    /**
     * Stops running all loops by calling their {@link Loop#onStop(double)} function.
     */
    public synchronized void stop() {
        if (running) {
            notifier.stop();
            synchronized (runningLock) {
                running = false;
                timestamp = Timer.getFPGATimestamp();
                for (Loop loop : loops) {
                    loop.onStop(timestamp);
                }
            }
        }
    }

    /**
     * Outputs telemetry to SmartDashboard/Shuffleboard about the current state of the looper.
     */
    public synchronized void outputTelemetry() {
        LOOPER_SHUFFLEBOARD.putNumber("Looper dt", dt);
    }
}