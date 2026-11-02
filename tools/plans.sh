#!/bin/bash
# Copies plans between the Control Hub and a folder on your computer.
#   tools/plans.sh pull [folder]   hub -> computer (then commit them, so a wiped hub can't lose them)
#   tools/plans.sh push [folder]   computer -> hub (after editing a file, or to restore a wiped hub)
# The folder defaults to ./recorded. Plug the hub in by USB, or join the robot's Wi-Fi. If a Driver Hub is plugged in,
# unplug it. Needs adb: on the PATH, in the Android SDK, or set with ADB=/path/to/adb.
ACTION="$1"
HERE="${2:-recorded}"
ADB="${ADB:-$(command -v adb || echo "$HOME/Library/Android/sdk/platform-tools/adb")}"
# The same storage has different names on different Android versions.
FOLDERS="/sdcard/recorded /storage/emulated/0/recorded /storage/self/primary/recorded"

die() { echo "ERROR: $1" >&2; exit 1; }
on_hub() { "$ADB" shell "$1" 2>/dev/null | tr -d '\r'; }
devices() { "$ADB" devices | sed 1d | awk '$2=="device"{print $1}'; }

case "$ACTION" in pull|push) ;; *) die "usage: plans.sh pull|push [folder]" ;; esac
[ -x "$ADB" ] || die "adb not found at $ADB. Set ADB=/path/to/adb"
if [ -z "$(devices)" ]; then
    echo "Nothing plugged in, trying the robot's Wi-Fi..."
    perl -e 'alarm 10; exec @ARGV' "$ADB" connect 192.168.43.1:5555 > /dev/null 2>&1
fi
COUNT=$(devices | wc -l | tr -d ' ')
[ "$COUNT" -gt 0 ] || die "can't see the hub. Plug it in by USB, or join the robot's Wi-Fi."
[ "$COUNT" -eq 1 ] || die "$COUNT devices connected. Unplug everything but the Control Hub: $(devices | tr '\n' ' ')"
echo "Connected to: $(on_hub 'getprop ro.product.model') ($(devices))"

# The folder that has plans in it, if any.
HUB=""
for f in $FOLDERS; do
    if [ -n "$(on_hub "ls $f/*.txt")" ]; then HUB="$f"; break; fi
done

n=0
if [ "$ACTION" = pull ]; then
    if [ -z "$HUB" ]; then
        echo "No plans found. What's in /sdcard on this device:" >&2
        on_hub 'ls /sdcard' >&2
        die "no plans in $FOLDERS. If this is the Driver Hub, plug in the Control Hub instead."
    fi
    mkdir -p "$HERE"
    for plan in $(on_hub "ls $HUB/*.txt"); do
        "$ADB" pull "$plan" "$HERE/" > /dev/null || die "couldn't copy $plan"
        n=$((n + 1))
    done
    echo "Copied $n plans from $HUB to $HERE"
else
    HUB="${HUB:-/sdcard/recorded}"
    "$ADB" shell mkdir -p "$HUB"
    for plan in "$HERE"/*.txt; do
        [ -f "$plan" ] || continue
        "$ADB" push "$plan" "$HUB/" > /dev/null || die "couldn't copy $plan"
        n=$((n + 1))
    done
    echo "Copied $n plans from $HERE to $HUB"
fi
