package org.firstinspires.ftc.teamcode.autorecorder;

import java.io.File;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

/*
 * No test framework, so ./test.sh needs only a JDK. Runs on a simulated robot, with no FTC SDK or hub.
 * Pass a folder of plan files to also replay each of them: ./test.sh path/to/recorded
 */
public final class PlanTests {
    private static int failures = 0;

    private static void check(String what, boolean ok) {
        System.out.println((ok ? "  ok    " : "  FAIL  ") + what);
        if (!ok) failures++;
    }

    private static void close(String what, double actual, double expected, double within) {
        check(String.format("%s: %.3f (want %.3f +- %.3f)", what, actual, expected, within),
                Math.abs(actual - expected) <= within);
    }

    /** A robot that does what the powers say, with a little lag, and full speed at FULL_SPEED inches / second. */
    private static final class SimRobot {
        static final double FULL_SPEED = 40, FULL_TURN = 6, LAG = 0.08;
        double x, y, heading, forward, right, turn;

        void step(PlanPlayer.Drive d, double dt) {
            double max = Math.max(1, Math.abs(d.forward) + Math.abs(d.right) + Math.abs(d.turn));
            double k = Math.min(1, dt / LAG);
            forward += (d.forward / max - forward) * k;
            right += (d.right / max - right) * k;
            turn += (d.turn / max - turn) * k;
            x += (forward * Math.cos(heading) + right * Math.sin(heading)) * FULL_SPEED * dt;
            y += (forward * Math.sin(heading) - right * Math.cos(heading)) * FULL_SPEED * dt;
            heading -= turn * FULL_TURN * dt;
        }
    }

    private static final class Result {
        double worstIn, endIn, seconds;
        boolean finished;
        final List<String> events = new ArrayList<>();
    }

    private static Result replay(Plan plan, PlanPlayer.Gate gate) {
        Result r = new Result();
        PlanPlayer player = new PlanPlayer(plan, e -> r.events.add(e.name + "@" + e.time));
        if (gate != null) player.setGate(gate);
        double[] first = plan.path.get(0);
        SimRobot robot = new SimRobot();
        robot.heading = first[3];
        double dt = 0.02, now = 0;
        while (!player.isDone() && now < plan.duration() + 15) {
            PlanPlayer.Drive d = player.update(now, robot.x, robot.y, robot.heading);
            double[] want = at(plan, player.planTime());
            r.worstIn = Math.max(r.worstIn, Math.hypot(want[1] - first[1] - robot.x, want[2] - first[2] - robot.y));
            robot.step(d, dt);
            now += dt;
        }
        double[] last = plan.path.get(plan.path.size() - 1);
        r.endIn = Math.hypot(last[1] - first[1] - robot.x, last[2] - first[2] - robot.y);
        r.seconds = now;
        r.finished = player.isDone();
        return r;
    }

    private static double[] at(Plan plan, double t) {
        double[] best = plan.path.get(0);
        for (double[] p : plan.path) if (p[0] <= t) best = p;
        return best;
    }

    /** A curving drive with a stop in the middle. */
    private static Plan curve() {
        Plan plan = new Plan("Curve", "TEST");
        for (int i = 0; i <= 80; i++) {
            double t = i * 0.05;
            double s = t < 2 ? t : t < 2.5 ? 2 : t - 0.5;
            plan.path.add(new double[]{t, 14 * s, 18 * Math.sin(s * 0.8), 1.5 * Math.sin(s * 0.5)});
        }
        plan.addEvent(1, "intake", "1.00");
        plan.addEvent(2.2, "shoot", "true");
        plan.addEvent(3, "intake", "0.00");
        return plan;
    }

    private static void store() throws Exception {
        System.out.println("PlanStore");
        File dir = Files.createTempDirectory("plans").toFile();
        PlanStore store = new PlanStore(dir);
        Plan plan = curve();
        check("saves", store.save(plan));
        check("exists", store.exists("Curve"));
        check("leaves no temp file", dir.list().length == 1);
        Plan back = store.loadAll().get(0);
        check("name and start survive", back.name.equals("Curve") && back.start.equals("TEST"));
        check("every point survives", back.path.size() == plan.path.size());
        check("every event survives", back.events.size() == 3 && back.events.get(1).value.equals("true"));

        Plan noStart = new Plan("No Start");
        noStart.path.add(new double[]{0, 0, 0, 0});
        noStart.addEvent(0, "flag", "");
        store.save(noStart);
        check("the start line is optional", PlanStore.load(new File(dir, "No_Start.txt")) != null);

        File hand = new File(dir, "Hand.txt");
        Files.write(hand.toPath(), ("// name Hand\np 0 0 0 0\ne 5 late\ne 1 early\np 1 10 0 0\n").getBytes());
        check("hand-edited events get sorted", PlanStore.load(hand).events.get(0).name.equals("early"));

        File bad = new File(dir, "BadEnd.txt");
        Files.write(bad.toPath(), ("// name BadEnd\np 0 0 0 0\np 1 30 0 0\np 2 60 0 0\np 2.05 0 0 0\n").getBytes());
        check("an encoder glitch at the end is cut", PlanStore.load(bad).path.size() == 3);

        File idle = new File(dir, "Idle.txt");
        Files.write(idle.toPath(), ("// name Idle\np 0 0 0 0\np 1 30 0 0\np 2 30 0 0\np 3 30 0 0\n").getBytes());
        check("the wait before STOP is cut", PlanStore.load(idle).path.size() == 2);

        File junk = new File(dir, "junk.txt");
        Files.write(junk.toPath(), "not a plan\n".getBytes());
        check("non-plans are skipped", PlanStore.load(junk) == null);

        int count = store.loadAll().size();
        Plan gone = store.loadAll().get(0);
        check("delete moves it", store.delete(gone) && new File(dir, "deleted").list().length == 1);
        check("deleted plans leave the list", store.loadAll().size() == count - 1);
    }

    private static void recorder() {
        System.out.println("PlanRecorder");
        Plan plan = new Plan("Rec");
        PlanRecorder rec = new PlanRecorder(plan);

        rec.tick(0.02);
        rec.track("flywheel", false);
        rec.trackPower("intake", 0);
        rec.trackSetting("rpm", 2000);
        rec.sample(0, 0, 0);
        rec.tick(5);
        rec.trackSetting("rpm", 2100);
        rec.sample(0.1, 0, 0);
        check("nothing happening keeps the clock at 0", !rec.hasStarted() && rec.seconds() == 0);

        rec.track("flywheel", true);
        check("a state change starts the clock", rec.hasStarted());
        rec.tick(1);
        close("time passes once started", rec.seconds(), 1, 1e-9);
        rec.trackPower("intake", 1);
        rec.tick(0.01);
        rec.trackPower("intake", 0.98);
        rec.tick(0.01);
        rec.trackPower("intake", 0.02);
        rec.tick(0.01);
        rec.trackPower("intake", 0);
        int intakes = 0;
        for (Plan.Event e : plan.events) if (e.name.equals("intake")) intakes++;
        check("small power changes are dropped, stopping never is (" + intakes + ")", intakes == 4);

        rec.setPaused(true);
        int before = plan.events.size();
        double was = rec.seconds();
        rec.tick(3);
        rec.track("flywheel", false);
        rec.sample(50, 50, 0);
        check("pausing stops the clock and the recording", rec.seconds() == was && plan.events.size() == before);
        rec.setPaused(false);

        Plan moved = new Plan("Moved");
        PlanRecorder mover = new PlanRecorder(moved);
        mover.sample(0, 0, 3.14);
        mover.sample(0, 0, -3.14);
        check("crossing +-180 degrees isn't a turn", !mover.hasStarted());
        mover.sample(1, 0, 3.14);
        check("moving starts the clock", mover.hasStarted());
    }

    private static void odometry() {
        System.out.println("MecanumOdometry");
        MecanumOdometry o = new MecanumOdometry(4, 100);
        double perTick = Math.PI * 4 / 100;
        o.reset();
        o.update(0, 0, 0, 0, 0);
        int forward = (int) Math.round(24 / perTick);
        o.update(forward, forward, forward, forward, 0);
        close("forward is +x", o.getX(), 24, 0.05);

        o = new MecanumOdometry(4, 100);
        o.update(0, 0, 0, 0, 0);
        o.update(forward, -forward, -forward, forward, 0);
        close("strafing right is -y", o.getY(), -24, 0.05);

        o = new MecanumOdometry(4, 100);
        o.update(0, 0, 0, 0, Math.PI / 2);
        o.update(forward, forward, forward, forward, Math.PI / 2);
        close("forward at 90 degrees is +y", o.getY(), 24, 0.05);

        o = new MecanumOdometry(4, 100);
        o.update(5000, 5000, 5000, 5000, 0);
        o.update(0, 0, 0, 0, 0);
        close("a bad read to 0 is ignored", o.getX(), 0, 1e-9);

        o = new MecanumOdometry(4, 100);
        o.update(0, 0, 0, 0, 0);
        o.update(0, 0, 0, 0, 2.5);
        close("a heading reset isn't a turn", o.getHeading(), 0, 1e-9);
    }

    private static void player() {
        System.out.println("PlanPlayer");
        Plan plan = curve();
        Result r = replay(plan, null);
        check("finishes", r.finished);
        close("ends on the last point", r.endIn, 0, 1.5);
        check(String.format("stays on the path (worst %.1f in)", r.worstIn), r.worstIn < 4);
        check("events fire in order", r.events.toString().equals("[intake@1.0, shoot@2.2, intake@3.0]"));

        double[] held = {0};
        Result gated = replay(plan, e -> {
            if (!e.name.equals("shoot")) return 0;
            held[0] += 0.02;
            return held[0] < 1.5 ? 10 : 0;   // ready after about 1.5 s
        });
        close("a gate holds the plan until it's ready", gated.seconds - r.seconds, 1.5, 0.3);
        close("the plan still ends on the last point", gated.endIn, 0, 1.5);

        Result stuck = replay(plan, e -> e.name.equals("shoot") ? 0.5 : 0);
        close("a gate that's never ready times out", stuck.seconds - r.seconds, 0.5, 0.3);

        Plan mirror = plan.mirrored();
        Result mirrored = replay(mirror, null);
        check("a mirrored plan still follows", mirrored.finished && mirrored.endIn < 1.5);
        double[] a = plan.path.get(40), b = mirror.path.get(40);
        close("mirroring flips sideways", b[2], -a[2], 1e-9);
        close("mirroring keeps forward", b[1], a[1], 1e-9);
        close("mirroring flips the turn", b[3], -a[3], 1e-9);
    }

    private static void folder(String path) {
        System.out.println("Plans in " + path);
        List<Plan> plans = new PlanStore(new File(path)).loadAll();
        check(plans.size() + " plans load", !plans.isEmpty());
        for (Plan plan : plans) {
            Result r = replay(plan, null);
            check(String.format("%-46s %4.1f s  worst %.1f in, end %.1f in", plan.name, plan.duration(), r.worstIn, r.endIn),
                    r.finished && r.endIn < 1.5 && r.events.size() == plan.events.size());
        }
    }

    public static void main(String[] args) throws Exception {
        store();
        recorder();
        odometry();
        player();
        if (args.length > 0) folder(args[0]);
        System.out.println(failures == 0 ? "All passed" : failures + " FAILED");
        System.exit(failures == 0 ? 0 : 1);
    }
}
