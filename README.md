# FTC Auto Recorder

Make an autonomous by driving it. Run **Auto Recorder**, drive the plan once with your normal controls, and press STOP.
**Auto Replay** then runs it: the robot's exact path (curves, diagonals, pushing into walls) and everything your mechanisms did, with the timing. There's nothing to paste or rebuild.

Replay follows the recorded position using the drive encoders and IMU, not the recorded motor powers, so a lower battery or a slipped wheel is corrected instead of copied.

- For the FTC SDK in Android Studio. Compiled against SDK 12.0.
- Needs a four-motor mecanum drive with encoders and an IMU, or your own localizer (see [Using it with your own code](#using-it-with-your-own-code)).
- Works for any season: it knows nothing about the game.

## Install

1. Download this repo.
2. Copy the `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autorecorder` folder into the same place in your project, next to your own code.
3. Open `autorecorder/MyRobot.java` and search for `CHANGE`. It's the only file you edit:
   - your motor names, directions, wheel size and hub mounting
   - your TeleOp loop in `teleop()`
   - each mechanism in `track()` and `onEvent()`
4. Build and run as usual. **Auto Recorder** appears under TeleOp and **Auto Replay** under Autonomous.

A mechanism that isn't in `track()` and `onEvent()` won't be replayed. For a claw:

```java
// in track(): what it's doing right now
recorder.track("claw", claw.getPosition() > 0.5 ? "OPEN" : "CLOSED");

// in onEvent(): how to do it again
case "claw": claw.setPosition(event.value.equals("OPEN") ? 1 : 0); break;
```

## Recording

Put the robot at its starting spot and run **Auto Recorder**.

| When | Control (gamepad1) | Does |
|---|---|---|
| INIT | D-pad up/down | Picks the plan number. Recording a number again replaces it. |
| INIT | Back / Share | Opens the edit menu to see and delete plans |
| Recording | Back / Share | Pauses and resumes. Paused time isn't in the plan. |
| Recording | STOP | Saves |

The clock starts at the first thing the robot does, so time before that isn't in the plan. To rename a plan, change the first line of its file.

## Replaying

Put the robot **where the recording started, facing the same way**, then run **Auto Replay**. D-pad up/down picks the plan. Replay stops at 30 s.

**The other alliance:** if the field rotates between alliances, the same plan works on both sides. If it mirrors, press **X** in INIT to mirror the plan.

### Tuning

The defaults are in `PlanPlayer.java`.

| Setting | What to do |
|---|---|
| `feedForwardSpeed` | Set it to the robot's real inches per second at full power |
| `feedForwardTurn` | The same for radians per second |
| `kP` | Raise it if the robot drifts off the path, lower it if it shakes |
| `leadTime` | Raise it if the robot overshoots stops or cuts corners, lower it if it slows too early |

Dead reckoning drifts, and a wheel that slips a lot won't be noticed. Measure `FORWARD_SCALE` and `STRAFE_SCALE` in `MyRobot` on a test drive, and square up by driving into a wall while recording.

### Waiting for a mechanism

A recording can't know your flywheel will take longer to spin up in a match. `holdFor()` in `MyRobot` makes an event wait, and the path stops until it's ready:

```java
public double holdFor(Plan.Event event) {
    if (event.name.equals("shoot") && !flywheel.isAtSpeed()) return 2;   // wait up to 2 s
    return 0;                                                            // ready
}
```

## Plans are text files

Plans live on the Control Hub in `/sdcard/recorded/`, one file each:

```text
// name Score Then Park
p 1.25 12.30 -4.00 1.5710    at 1.25 s the robot is here: x in, y in, heading radians
e 1.30 intake 1.00           at 1.30 s this happens
```

You can edit `e` lines by hand: change the time, or delete one.

**Back them up.** Copy plans to your computer and commit them, because a wiped hub loses them. `plans.sh` needs `adb`, which comes with Android Studio:

```bash
tools/plans.sh pull
```

```bash
tools/plans.sh push
```

**See a plan:** open `viewer/plan-viewer.html` in a browser and drop plan files on it. It draws the path with a timeline you can play or scrub. You can load a picture of this season's field and drag the robot to its start. It needs no internet or install.

## Using it with your own code

`MyRobot` and the two OpModes are a starting point. The rest has no FTC or game dependencies, so you can call it from your own OpModes:

| Class | Does |
|---|---|
| `Plan` | A recorded path and events |
| `PlanStore` | Saves and loads plans |
| `PlanRecorder` | Turns a TeleOp loop into a plan |
| `PlanPlayer` | Turns a plan into drive powers, each loop |
| `MecanumOdometry` | Pose from four drive encoders and an IMU |

Using odometry pods, Pinpoint or a vision localizer? Skip `MecanumOdometry` and pass your own pose to `PlanRecorder.sample()` and `PlanPlayer.update()`. They need x along heading 0 and y to its left, in inches, with heading in radians, counter-clockwise positive.

`PlanPlayer.setHeadingOverride()` aims with a camera or other sensor in place of the recorded heading, until you set it back to `null`.

## Tests

`./test.sh` needs only a JDK. It runs the library on a simulated robot, with no SDK or hub.

## License

MIT. See [LICENSE](LICENSE).
