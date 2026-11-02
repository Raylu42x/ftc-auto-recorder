package org.firstinspires.ftc.teamcode.autorecorder.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autorecorder.MecanumOdometry;
import org.firstinspires.ftc.teamcode.autorecorder.MyRobot;
import org.firstinspires.ftc.teamcode.autorecorder.Plan;
import org.firstinspires.ftc.teamcode.autorecorder.PlanPlayer;
import org.firstinspires.ftc.teamcode.autorecorder.PlanStore;

import java.util.List;
import java.util.Locale;

/*
 * Plays a plan from Auto Recorder. Pick one in INIT, put the robot where the recording started, press START.
 *
 * INIT (gamepad1):   D-pad up/down = pick a plan     X = mirror it
 *
 * Fields that rotate between alliances need nothing: the same plan works on both sides. Fields that mirror
 * need X on the other alliance. Replay stops at the end of the 30 s auto period.
 */
@Autonomous(name = "Auto Replay", group = "Auto")
public class AutoReplay extends LinearOpMode {
    @Override
    public void runOpMode() {
        MyRobot robot = new MyRobot(hardwareMap);
        MecanumOdometry odometry = robot.newOdometry();
        List<Plan> plans = new PlanStore(AutoRecorderTeleOp.FOLDER).loadAll();

        int choice = 0;
        boolean mirror = false;
        Gamepad prev = new Gamepad();
        while (opModeInInit()) {
            Gamepad g = gamepad1;
            if (!plans.isEmpty()) {
                if (g.dpad_down && !prev.dpad_down) choice = (choice + 1) % plans.size();
                if (g.dpad_up && !prev.dpad_up) choice = (choice + plans.size() - 1) % plans.size();
            }
            if (g.x && !prev.x) mirror = !mirror;
            prev.copy(g);

            if (plans.isEmpty()) {
                telemetry.addLine("No plans saved. Record one with Auto Recorder.");
            } else {
                telemetry.addLine("Plan: " + plans.get(choice).name + "   (D-pad up/down)");
                telemetry.addLine(String.format(Locale.US, "%.1f s", plans.get(choice).duration()));
                telemetry.addLine("Mirrored: " + (mirror ? "YES" : "NO") + "   (X)");
            }
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested() || plans.isEmpty()) return;

        Plan plan = mirror ? plans.get(choice).mirrored() : plans.get(choice);
        robot.setHeading(plan.startHeading());
        odometry.reset();
        robot.update(odometry);

        PlanPlayer player = new PlanPlayer(plan, robot::onEvent);
        player.setGate(robot::holdFor);
        ElapsedTime clock = new ElapsedTime();
        while (opModeIsActive() && !player.isDone() && clock.seconds() < 30) {
            robot.update(odometry);
            PlanPlayer.Drive d = player.update(clock.seconds(), odometry.getX(), odometry.getY(), odometry.getHeading());
            robot.drive(d.forward, d.right, d.turn);
            telemetry.addLine(plan.name);
            telemetry.addLine(String.format(Locale.US, "%.1f of %.1f s", player.planTime(), plan.duration()));
            telemetry.update();
        }
        robot.stop();
    }
}
