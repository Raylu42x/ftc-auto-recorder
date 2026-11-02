package org.firstinspires.ftc.teamcode.autorecorder;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * THE ONE FILE TO EDIT. Everything Auto Recorder and Auto Replay need to know about your robot is here. It works as is
 * for a mecanum drive. Search for "CHANGE" and follow the three steps:
 *
 *   1. Your hardware: motor names, directions, hub mounting and wheel size.
 *   2. teleop(): your normal driving and mechanism code, so recording feels like a match.
 *   3. track() and onEvent(): what your mechanisms are doing, and how to do it again. A mechanism missing from
 *      these two won't be replayed. The intake below shows the pattern; copy it for each mechanism.
 */
public class MyRobot {
    // CHANGE: wheel size, encoder ticks per wheel turn (motor's ticks x gearing), hub mounting.
    static final double WHEEL_DIAMETER_IN = 4.09;
    static final double TICKS_PER_REV = 537.7;
    static final double FORWARD_SCALE = 1, STRAFE_SCALE = 1;   // measured distance / asked distance on a test drive
    static final RevHubOrientationOnRobot.LogoFacingDirection LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection USB = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;
    private double headingOffset = 0;

    private final DcMotor intake;   // CHANGE: your mechanisms

    public MyRobot(HardwareMap hardwareMap) {
        // CHANGE: names from your robot configuration, and flip a direction if that wheel spins backwards.
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx motor : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LOGO, USB)));
        imu.resetYaw();

        intake = hardwareMap.get(DcMotor.class, "intake");   // CHANGE
    }

    /** CHANGE: your TeleOp's loop body. Runs every loop while recording. */
    public void teleop(Gamepad gamepad1, Gamepad gamepad2) {
        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
    }

    /**
     * CHANGE: report what each mechanism is doing right now, every loop. Changes become events.
     *   recorder.trackPower("intake", power)    a motor power
     *   recorder.track("claw", "OPEN")          a state: a word or true / false
     *   recorder.trackSetting("lift", target)   a number that shouldn't start the clock by itself
     */
    public void track(PlanRecorder recorder) {
        recorder.trackPower("intake", intake.getPower());
    }

    /** CHANGE: do one recorded event. Use the same names as track(). */
    public void onEvent(Plan.Event event) {
        switch (event.name) {
            case "intake": intake.setPower(Double.parseDouble(event.value)); break;
            default: break;
        }
    }

    /**
     * CHANGE (optional): make an event wait for the robot, e.g. a shot for the flywheel. The path stops meanwhile.
     * Return the most seconds it may wait for, or 0 when it's ready to go.
     */
    public double holdFor(Plan.Event event) {
        return 0;
    }

    /** CHANGE (optional): turn everything off. */
    public void stop() {
        drive(0, 0, 0);
        intake.setPower(0);
    }

    // ---- the drive: only change this if you don't have a four-motor mecanum drive ----

    /** Robot-centric. forward +, right +, turn + is clockwise. */
    public void drive(double forward, double right, double turn) {
        double fl = forward + right + turn, bl = forward - right + turn;
        double fr = forward - right - turn, br = forward + right - turn;
        double max = Math.max(1, Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br))));
        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    /** Radians, counter-clockwise positive. */
    public double getHeading() {
        return MecanumOdometry.angleWrap(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + headingOffset);
    }

    /** Tells the robot it's currently facing this heading. */
    public void setHeading(double radians) {
        imu.resetYaw();
        headingOffset = radians;
    }

    public MecanumOdometry newOdometry() {
        MecanumOdometry odometry = new MecanumOdometry(WHEEL_DIAMETER_IN, TICKS_PER_REV);
        odometry.forwardScale = FORWARD_SCALE;
        odometry.strafeScale = STRAFE_SCALE;
        return odometry;
    }

    public void update(MecanumOdometry odometry) {
        odometry.update(frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(), backRight.getCurrentPosition(), getHeading());
    }
}
