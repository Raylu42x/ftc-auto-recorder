package org.firstinspires.ftc.teamcode.autorecorder;

/*
 * Where the robot is, from the four drive encoders + IMU (dead reckoning). x is along heading 0, y to its left, inches
 * from where it started. Call update() every loop. Using odometry pods or another localizer? Skip this class and give
 * PlanRecorder and PlanPlayer your own pose: both only need x, y and heading in this frame.
 *
 * The heading ignores a sudden jump (someone reset the heading), so a reset mid-recording doesn't break the path.
 */
public class MecanumOdometry {
    /** A heading change bigger than this in one loop isn't a turn, it's a heading reset. */
    private static final double HEADING_JUMP_RAD = 1.0;

    /** Measured distance / asked distance from a test drive. Slipping mecanum rollers rarely make these 1. */
    public double forwardScale = 1, strafeScale = 1;
    /**
     * A wheel can't turn this many ticks in one loop. It means a bad read, e.g. the hub returning 0 as the OpMode
     * stops, which would look like the robot jumping back to where it started.
     */
    public int maxTicksPerLoop = 900;

    private final double inchesPerTick;
    private final int[] lastTicks = new int[4];
    private boolean started = false;
    private double lastRaw;
    private double heading, x, y;

    public MecanumOdometry(double wheelDiameterIn, double ticksPerRev) {
        inchesPerTick = Math.PI * wheelDiameterIn / ticksPerRev;
    }

    /** Start from here: position 0, 0. The next update() takes its heading from the IMU. */
    public void reset() {
        started = false;
        x = 0;
        y = 0;
    }

    /** Wheels are positive when the robot goes forward. Heading in radians, counter-clockwise positive. */
    public void update(int frontLeft, int frontRight, int backLeft, int backRight, double raw) {
        int[] t = {frontLeft, frontRight, backLeft, backRight};
        if (!started) {
            System.arraycopy(t, 0, lastTicks, 0, 4);
            lastRaw = raw;
            heading = raw;
            started = true;
            return;
        }
        double fl = t[0] - lastTicks[0], fr = t[1] - lastTicks[1], bl = t[2] - lastTicks[2], br = t[3] - lastTicks[3];
        if (Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))) > maxTicksPerLoop) return;
        System.arraycopy(t, 0, lastTicks, 0, 4);
        double turned = angleWrap(raw - lastRaw);
        lastRaw = raw;
        if (Math.abs(turned) > HEADING_JUMP_RAD) turned = 0;

        double forward = (fl + bl + fr + br) / 4 * inchesPerTick * forwardScale;
        double right = (fl - bl - fr + br) / 4 * inchesPerTick * strafeScale;
        double mid = heading + turned / 2;   // heading halfway through this loop's move
        heading += turned;
        x += forward * Math.cos(mid) + right * Math.sin(mid);
        y += forward * Math.sin(mid) - right * Math.cos(mid);
    }

    public double getX() { return x; }
    public double getY() { return y; }
    /** Radians, counter-clockwise positive. Not wrapped: a recording can turn past 180 degrees smoothly. */
    public double getHeading() { return heading; }

    public static double angleWrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }
}
