package org.firstinspires.ftc.teamcode.autorecorder.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.autorecorder.MecanumOdometry;
import org.firstinspires.ftc.teamcode.autorecorder.MyRobot;
import org.firstinspires.ftc.teamcode.autorecorder.Plan;
import org.firstinspires.ftc.teamcode.autorecorder.PlanRecorder;
import org.firstinspires.ftc.teamcode.autorecorder.PlanStore;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/*
 * Drive a plan once with your normal controls. It records the robot's exact path and everything the mechanisms do,
 * with the timing. "Auto Replay" then runs it. Nothing to paste or rebuild.
 *
 * INIT (gamepad1):   D-pad up/down = plan number     Back / Share = edit menu
 * EDIT MENU:         D-pad up/down = pick     A = delete (A again to confirm, B = no)     Back / Share = return
 * RECORDING:         Back / Share = pause / resume     STOP = save
 *
 * The clock starts at the first thing the robot does. Put the robot in the same spot before replaying: a plan is
 * relative to where it started. To rename a plan, change the first line of its file.
 */
@TeleOp(name = "Auto Recorder", group = "Auto")
public class AutoRecorderTeleOp extends LinearOpMode {
    static final File FOLDER = new File(AppUtil.ROOT_FOLDER, "recorded");
    private static final int EDIT_ROWS = 10;

    private final PlanStore store = new PlanStore(FOLDER);
    private boolean editing = false, confirmDelete = false;
    private List<Plan> plans = new ArrayList<>();
    private int cursor = 0;
    private String editNote = "";

    @Override
    public void runOpMode() {
        MyRobot robot = new MyRobot(hardwareMap);
        MecanumOdometry odometry = robot.newOdometry();

        int number = 1;
        Gamepad prev = new Gamepad();
        while (opModeInInit()) {
            Gamepad g = gamepad1;
            if (g.back && !prev.back) {
                editing = !editing;
                plans = store.loadAll();
                cursor = 0;
                confirmDelete = false;
                editNote = "";
            }
            if (editing) {
                editMenu(g, prev);
            } else {
                if (g.dpad_up && !prev.dpad_up) number++;
                if (g.dpad_down && !prev.dpad_down) number = Math.max(1, number - 1);
                telemetry.addLine("AUTO RECORDER");
                telemetry.addLine("Plan " + number + "   (D-pad up/down)");
                telemetry.addLine("Back / Share = see and delete plans");
                if (store.exists("Plan " + number)) telemetry.addLine("Already recorded: this replaces it");
                telemetry.update();
            }
            prev.copy(g);
            sleep(20);
        }
        if (isStopRequested()) return;

        Plan plan = new Plan("Plan " + number);
        PlanRecorder recorder = new PlanRecorder(plan);
        robot.setHeading(0);
        odometry.reset();
        robot.update(odometry);

        boolean prevBack = false;
        String saveNote = "";
        ElapsedTime loop = new ElapsedTime();

        while (opModeIsActive()) {
            double dt = loop.seconds();
            loop.reset();
            boolean back = gamepad1.back;
            if (back && !prevBack) {
                recorder.setPaused(!recorder.isPaused());
                if (recorder.isPaused() && recorder.hasStarted()) {
                    saveNote = store.save(plan) ? "" : "COULDN'T SAVE THE PLAN";
                }
            }
            prevBack = back;

            if (recorder.isPaused()) {
                robot.drive(0, 0, 0);
                robot.update(odometry);
                telemetry.addLine("PAUSED   Back / Share = resume");
                telemetry.addLine(String.format(Locale.US, "%s  %.1f s so far", plan.name, recorder.seconds()));
                telemetry.addLine(saveNote);
                telemetry.update();
                continue;
            }

            recorder.tick(dt);
            robot.teleop(gamepad1, gamepad2);
            robot.update(odometry);
            if (!opModeIsActive()) break;   // STOP: reads may be bad now, so don't record this loop

            robot.track(recorder);
            recorder.sample(odometry.getX(), odometry.getY(), odometry.getHeading());

            telemetry.addLine(recorder.hasStarted()
                    ? String.format(Locale.US, "REC %.1f s   Back / Share = pause, STOP = save", recorder.seconds())
                    : "READY: starts when the robot does");
            telemetry.addLine(plan.name);
            if (recorder.seconds() > 30) telemetry.addLine("Over 30 s: auto ends before this does");
            telemetry.update();
        }
        robot.stop();
        if (recorder.hasStarted()) store.save(plan);   // never replace a plan with a run where nothing happened
    }

    /** Lists every saved plan; A deletes the picked one, after a second A. Deleted plans go to recorded/deleted/. */
    private void editMenu(Gamepad g, Gamepad prev) {
        if (!plans.isEmpty() && !confirmDelete) {
            if (g.dpad_down && !prev.dpad_down) cursor = (cursor + 1) % plans.size();
            if (g.dpad_up && !prev.dpad_up) cursor = (cursor + plans.size() - 1) % plans.size();
        }
        if (confirmDelete && g.b && !prev.b) confirmDelete = false;
        if (!plans.isEmpty() && g.a && !prev.a) {
            if (!confirmDelete) {
                confirmDelete = true;
            } else {
                Plan gone = plans.get(cursor);
                editNote = (store.delete(gone) ? "Deleted: " : "COULDN'T DELETE: ") + gone.name;
                plans = store.loadAll();
                cursor = Math.max(0, Math.min(cursor, plans.size() - 1));
                confirmDelete = false;
            }
        }

        telemetry.addLine("EDIT PLANS   " + plans.size() + " saved");
        if (plans.isEmpty()) telemetry.addLine("No plans saved.");
        int first = Math.max(0, Math.min(cursor - EDIT_ROWS / 2, plans.size() - EDIT_ROWS));
        for (int i = first; i < Math.min(plans.size(), first + EDIT_ROWS); i++) {
            telemetry.addLine((i == cursor ? " >  " : "     ") + plans.get(i).name
                    + String.format(Locale.US, "  (%.0f s)", plans.get(i).duration()));
        }
        telemetry.addLine(confirmDelete ? "Delete this plan? A = YES, B = no"
                : "D-pad up/down = pick | A = delete | Back / Share = return");
        if (!editNote.isEmpty()) telemetry.addLine(editNote);
        telemetry.update();
    }
}
