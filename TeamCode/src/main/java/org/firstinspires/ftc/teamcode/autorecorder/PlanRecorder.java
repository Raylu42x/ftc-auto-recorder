package org.firstinspires.ftc.teamcode.autorecorder;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

/*
 * Turns a normal TeleOp into a recording. Every loop, in this order:
 *
 *   recorder.tick(secondsSinceLastLoop);
 *   ... your usual driving and mechanism code ...
 *   recorder.track("flywheel", shooter.isSpinning());      // what the robot is doing right now
 *   recorder.trackPower("intake", intake.getPower());
 *   recorder.trackSetting("rpm", Math.round(shooter.getTargetRpm()));
 *   recorder.sample(x, y, heading);
 *
 * Changes become events. The clock stays at 0 until the robot does something, so time before that isn't in the plan.
 * While paused the clock stops and nothing is recorded.
 */
public class PlanRecorder {
    /** Seconds between saved path points (20 a second). */
    public double sampleTime = 0.05;
    /** Moving this far (inches) or turning this far (degrees) starts the clock. */
    public double startMoveIn = 0.25;
    public double startTurnDeg = 1;
    /** Power changes smaller than this aren't saved. */
    public double powerStep = 0.05;

    public final Plan plan;

    private final Map<String, String> last = new HashMap<>();
    private final Map<String, Double> lastPower = new HashMap<>();
    private boolean started = false, paused = false, hasStart = false;
    private double clock = 0, nextSample = 0;
    private double startX, startY, startHeading;

    public PlanRecorder(Plan plan) {
        this.plan = plan;
    }

    /** Call first in every loop, with the seconds since the last one. */
    public void tick(double seconds) {
        if (started && !paused) clock += seconds;
    }

    public void setPaused(boolean paused) {
        this.paused = paused;
    }

    public boolean isPaused() { return paused; }
    /** False until the robot does something. Don't save a plan that never started: nothing happened in it. */
    public boolean hasStarted() { return started; }
    /** Seconds recorded so far. */
    public double seconds() { return clock; }

    /** Records this state when it changes. Any change after the first value starts the clock. */
    public void track(String name, String value) {
        record(name, value, true);
    }

    public void track(String name, boolean value) {
        track(name, String.valueOf(value));
    }

    /** Records this number when it changes, without starting the clock: a camera nudging a target isn't the robot doing something. */
    public void trackSetting(String name, double value) {
        record(name, format(value), false);
    }

    /** Records a motor power when it changes by powerStep, or stops. Any change starts the clock. */
    public void trackPower(String name, double power) {
        if (paused) return;
        Double before = lastPower.get(name);
        if (before != null && Math.abs(power - before) < powerStep && (power != 0 || before == 0)) return;
        lastPower.put(name, power);
        record(name, String.format(Locale.US, "%.2f", power), true);
    }

    private void record(String name, String value, boolean startsClock) {
        if (paused) return;
        String before = last.put(name, value);
        if (value.equals(before)) return;
        if (before != null && startsClock) started = true;
        plan.addEvent(clock, name, value);
    }

    private static String format(double value) {
        return value == Math.rint(value) ? String.valueOf(Math.round(value)) : String.format(Locale.US, "%.3f", value);
    }

    /** Call last in every loop, with where the robot is (see MecanumOdometry, or your own localizer). */
    public void sample(double x, double y, double heading) {
        if (paused) return;
        if (!hasStart) {
            hasStart = true;
            startX = x;
            startY = y;
            startHeading = heading;
        }
        started = started || Math.hypot(x - startX, y - startY) > startMoveIn
                || Math.abs(MecanumOdometry.angleWrap(heading - startHeading)) > Math.toRadians(startTurnDeg);
        if (clock >= nextSample) {
            plan.path.add(new double[]{clock, x, y, heading});
            nextSample = clock + sampleTime;
        }
    }
}
