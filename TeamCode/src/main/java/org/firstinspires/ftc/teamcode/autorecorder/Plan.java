package org.firstinspires.ftc.teamcode.autorecorder;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/*
 * A recorded plan: the robot's exact path and everything the mechanisms did, with the times.
 * PlanRecorder makes one, PlanStore saves it as text, PlanPlayer drives it.
 *
 *   // name Score Then Park     <- change this line to rename the plan
 *   // start LEFT_WALL          <- optional label: whatever helps you put the robot in the right place
 *   p 1.25 12.3 -4.0 1.571      <- at 1.25 s the robot is here: x in, y in, heading radians
 *   e 1.30 intake 1.00          <- at 1.30 s this happens: event name, optional value
 *
 * x is along heading 0, y to its left, inches from where the robot started. Heading is counter-clockwise positive.
 * "e" lines can be edited by hand; change their time or delete them.
 */
public final class Plan {
    /** Faster than a robot can go (inches / second): a path point like this is a bad encoder read. */
    private static final double IMPOSSIBLE_SPEED = 120;

    public static final class Event {
        public final double time;
        public final String name, value;
        Event(double time, String name, String value) {
            this.time = time;
            this.name = name;
            this.value = value;
        }
    }

    public String name;
    public String start;
    public final List<double[]> path = new ArrayList<>();   // {t, x, y, heading}
    public final List<Event> events = new ArrayList<>();

    public Plan(String name) {
        this(name, null);
    }

    public Plan(String name, String start) {
        this.name = name;
        this.start = start;
    }

    public void addEvent(double time, String name, String value) {
        events.add(new Event(time, name, value));
    }

    public double duration() {
        double end = path.isEmpty() ? 0 : path.get(path.size() - 1)[0];
        return events.isEmpty() ? end : Math.max(end, events.get(events.size() - 1).time);
    }

    /** The heading the robot had when recording started. Tell your IMU this before replaying. */
    public double startHeading() {
        return path.isEmpty() ? 0 : path.get(0)[3];
    }

    /**
     * This plan reflected left-right about the start line, for a field that mirrors between alliances instead of
     * rotating. Starts in the same place at the same heading; left turns become right turns. Events don't change.
     */
    public Plan mirrored() {
        Plan out = new Plan(name, start);
        if (path.isEmpty()) return out;
        double[] first = path.get(0);
        double c = Math.cos(first[3]), s = Math.sin(first[3]);
        for (double[] p : path) {
            double dx = p[1] - first[1], dy = p[2] - first[2];
            double along = dx * c + dy * s, left = dy * c - dx * s;
            out.path.add(new double[]{p[0], first[1] + along * c + left * s, first[2] + along * s - left * c,
                    2 * first[3] - p[3]});
        }
        out.events.addAll(events);
        return out;
    }

    /** What PlanStore does to a plan after reading it, so replay never sees a bad or idle end. */
    void tidy() {
        Collections.sort(events, new Comparator<Event>() {   // stable, so hand edits can't block the plan
            @Override public int compare(Event a, Event b) { return Double.compare(a.time, b.time); }
        });
        dropBadEnd();
        trimEnd();
    }

    /**
     * Cuts the path at the first impossible jump. Plans can end with one: as the OpMode stopped, the encoders could
     * read 0, so the last point is back near the start, and replay would drive all the way there.
     */
    private void dropBadEnd() {
        for (int i = 1; i < path.size(); i++) {
            double[] a = path.get(i - 1), b = path.get(i);
            double seconds = Math.max(0.01, b[0] - a[0]);
            if (Math.hypot(b[1] - a[1], b[2] - a[2]) / seconds > IMPOSSIBLE_SPEED) {
                while (path.size() > i) path.remove(path.size() - 1);
                return;
            }
        }
    }

    /**
     * Drops the end of the recording where nothing happens (the wait before someone pressed STOP), so replay
     * doesn't sit on the last point correcting small errors for no reason.
     */
    private void trimEnd() {
        double[] last = path.get(path.size() - 1);
        double lastEvent = events.isEmpty() ? 0 : events.get(events.size() - 1).time;
        int keep = path.size() - 1;
        while (keep > 0) {
            double[] p = path.get(keep - 1);
            boolean sameSpot = Math.hypot(p[1] - last[1], p[2] - last[2]) < 0.25 && Math.abs(p[3] - last[3]) < 0.02;
            if (!sameSpot || p[0] < lastEvent) break;
            keep--;
        }
        while (path.size() > keep + 1) path.remove(path.size() - 1);
    }
}
