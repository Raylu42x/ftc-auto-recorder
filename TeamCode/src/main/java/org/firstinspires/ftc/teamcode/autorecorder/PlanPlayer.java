package org.firstinspires.ftc.teamcode.autorecorder;

import static org.firstinspires.ftc.teamcode.autorecorder.MecanumOdometry.angleWrap;

import java.util.List;

/*
 * Plays a plan: follows the recorded path at the recorded speed, and hands each event to your code at its time.
 * Position comes from your pose every loop, so a low battery or a slipped wheel is corrected, not copied.
 *
 *   PlanPlayer player = new PlanPlayer(plan, event -> ...);
 *   ElapsedTime clock = new ElapsedTime();
 *   while (opModeIsActive() && !player.isDone()) {
 *       odometry.update(...);
 *       PlanPlayer.Drive d = player.update(clock.seconds(), odometry.getX(), odometry.getY(), odometry.getHeading());
 *       yourMecanumDrive(d.forward, d.right, d.turn);
 *   }
 *   stop the motors
 *
 * Give the IMU plan.startHeading() and reset your pose to 0, 0 first. Tune: set feedForwardSpeed to how fast the robot
 * really goes at full power, then raise kP if it drifts off the path and lower it if it shakes.
 */
public class PlanPlayer {
    /** Does one event: run the intake, move a servo. */
    public interface EventHandler {
        void onEvent(Plan.Event event);
    }

    /** Lets an event wait for the robot, e.g. a shot for the flywheel. The path stops and the plan's clock holds. */
    public interface Gate {
        /** How many seconds in all this event may hold the plan up. 0 = go now. */
        double holdFor(Plan.Event event);
    }

    /** Powers for one loop. Same meaning as gamepad sticks: forward +, right +, clockwise +. Not normalized. */
    public static final class Drive {
        public double forward, right, turn;
    }

    /** Seconds either side of a path point used to work out its speed (smooths out the 20-a-second samples). */
    private static final double VELOCITY_SPAN = 0.05;

    /** Inches per second the robot goes at full power (the recorded speed is turned into power with this). */
    public double feedForwardSpeed = 40;
    /** Radians per second the robot turns at full power. */
    public double feedForwardTurn = 6;
    /** Extra power per inch off the recorded path. */
    public double kP = 0.12;
    /** Extra turn power per radian off the recorded heading. */
    public double turnKp = 1.2;
    /** Smallest turn power that moves the robot, used while setHeadingOverride aims. */
    public double turnMinPower = 0.12;
    /**
     * Seconds ahead the speed is read, so the wheels are already changing when the path does. Raise if the robot
     * overshoots stops or cuts corners, lower if it slows or turns too early.
     */
    public double leadTime = 0.15;
    /** At the end: this close counts as there. Otherwise it gives up after endTime seconds. */
    public double endToleranceIn = 1.5, endToleranceDeg = 1, endTime = 1;
    /** While overriding the heading, errors smaller than this don't get turnMinPower. */
    public double overrideToleranceDeg = 0.75;

    private final Plan plan;
    private final List<double[]> path;
    private final EventHandler handler;
    private final Drive out = new Drive();
    private final double offX, offY;
    private Gate gate;
    private Double headingOverride;
    private int index = 0, nextEvent = 0;
    private double held = 0, waited = 0, lastTime = 0, planTime = 0;
    private double endedAt = -1;
    private boolean done = false, first = true;

    public PlanPlayer(Plan plan, EventHandler handler) {
        this.plan = plan;
        this.path = plan.path;
        this.handler = handler;
        offX = path.get(0)[1];
        offY = path.get(0)[2];
    }

    public void setGate(Gate gate) {
        this.gate = gate;
    }

    /** Aim at this heading instead of the recorded one, e.g. a camera lock-on, until set back to null. */
    public void setHeadingOverride(Double heading) {
        headingOverride = heading;
    }

    public boolean isDone() { return done; }
    /** Where the plan is up to, in its own seconds. */
    public double planTime() { return planTime; }

    /** Call every loop. now is seconds since replay started, x / y / heading are the robot's pose. */
    public Drive update(double now, double x, double y, double heading) {
        double dt = first ? 0 : now - lastTime;
        first = false;
        lastTime = now;
        double t = now - held;

        // ---- events up to now; a gate can hold the plan ----
        boolean waiting = false;
        while (nextEvent < plan.events.size() && plan.events.get(nextEvent).time <= t) {
            Plan.Event e = plan.events.get(nextEvent);
            if (gate != null && waited < gate.holdFor(e)) {
                waited += dt;
                held += dt;
                t = e.time;
                waiting = true;
                break;
            }
            waited = 0;
            handler.onEvent(e);
            nextEvent++;
        }
        planTime = t;

        // ---- where the recording says to be ----
        while (index + 1 < path.size() && path.get(index + 1)[0] <= t) index++;
        double[] a = path.get(index), b = path.get(Math.min(index + 1, path.size() - 1));
        double span = b[0] - a[0];
        double k = span > 0 ? Math.max(0, Math.min(1, (t - a[0]) / span)) : 0;
        double targetX = a[1] + (b[1] - a[1]) * k - offX, targetY = a[2] + (b[2] - a[2]) * k - offY;
        double targetHeading = a[3] + angleWrap(b[3] - a[3]) * k;
        // Recorded speed, read a little ahead. Past the end it's 0, so the robot starts stopping early.
        // Waiting at a gate, the path isn't moving.
        double[] before = poseAt(t + leadTime - VELOCITY_SPAN), after = poseAt(t + leadTime + VELOCITY_SPAN);
        double velScale = waiting ? 0 : 1 / (2 * VELOCITY_SPAN);
        double velX = (after[0] - before[0]) * velScale, velY = (after[1] - before[1]) * velScale;
        double velTurn = angleWrap(after[2] - before[2]) * velScale;

        if (headingOverride != null) {
            targetHeading = headingOverride;
            velTurn = 0;
        }

        // ---- drive there: the recorded speed plus a correction for any error, in the robot's frame ----
        double powerX = velX / feedForwardSpeed + (targetX - x) * kP;
        double powerY = velY / feedForwardSpeed + (targetY - y) * kP;
        double error = angleWrap(targetHeading - heading);
        double turn = -(velTurn / feedForwardTurn + error * turnKp);
        // Small aim errors ask for too little power to move the robot, so give an override a minimum.
        if (headingOverride != null && Math.abs(error) > Math.toRadians(overrideToleranceDeg)
                && Math.abs(turn) < turnMinPower) {
            turn = -Math.signum(error) * turnMinPower;
        }
        out.forward = powerX * Math.cos(heading) + powerY * Math.sin(heading);
        out.right = powerX * Math.sin(heading) - powerY * Math.cos(heading);
        out.turn = Math.max(-1, Math.min(1, turn));

        // ---- finished: a moment to settle on the last point ----
        if (t >= plan.duration() && nextEvent >= plan.events.size()) {
            if (endedAt < 0) endedAt = now;
            boolean there = Math.hypot(targetX - x, targetY - y) < endToleranceIn
                    && Math.abs(error) < Math.toRadians(endToleranceDeg);
            done = there || now - endedAt > endTime;
        }
        return out;
    }

    /** The path at time t: {x, y, heading}, between samples, held at the ends. */
    private double[] poseAt(double t) {
        int low = 0, high = path.size() - 1;
        while (high - low > 1) {
            int mid = (low + high) / 2;
            if (path.get(mid)[0] <= t) low = mid; else high = mid;
        }
        double[] a = path.get(low), b = path.get(high);
        double span = b[0] - a[0];
        double k = span > 0 ? Math.max(0, Math.min(1, (t - a[0]) / span)) : 0;
        return new double[]{a[1] + (b[1] - a[1]) * k, a[2] + (b[2] - a[2]) * k, a[3] + (b[3] - a[3]) * k};
    }
}
