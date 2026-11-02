#!/bin/bash
# Compiles the library as Java 8 (what the FTC SDK builds with) and runs the tests. Needs only a JDK, not the SDK.
#   ./test.sh                     the tests
#   ./test.sh path/to/recorded    also replays each plan in that folder on the simulated robot
set -e
cd "$(dirname "$0")"
SRC=TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autorecorder
OUT=build/test
rm -rf "$OUT" && mkdir -p "$OUT"
javac --release 8 -Xlint:all,-options -Werror -d "$OUT/main" "$SRC"/Plan*.java "$SRC"/MecanumOdometry.java
javac -cp "$OUT/main" -d "$OUT/tests" src/test/java/autorecorder/*.java
java -cp "$OUT/main:$OUT/tests" org.firstinspires.ftc.teamcode.autorecorder.PlanTests "$@"
