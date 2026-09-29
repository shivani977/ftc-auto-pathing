# ftc-auto-pathing


Autonomous paths for our FTC robot. Made on Pedro Pathing with AprilTag detection and Panels telemetry.

Files:

BIG_RED.java - simple red auto, 3 paths, fixed start
redddd.java - red auto, picks paths based on AprilTag (21/22 vs 23)
Constants.java - drivetrain, localizer, follower config
Tuning.java - Pedro tuning OpModes

BIG_RED

Starts at (122, 123), goes to (72,72), out to (15,36), back. Intake and outtake run on timers. Note that STATE_START is unreachable right now, launch jumps straight to finish. Need to fix that.

redddd

Reads a tag during init, then runs one of two 6-path sequences: launch, pickup, launch, pickup, launch, turn. No tag means no movement. Start pose is (76, 9), max power 0.65.

Hardware

Mecanum drive, motors fl fr bl br with left side reversed. Drive encoders only, no dead wheels. Intake is "i", outtake is "f1"/"f2" in redddd (check BIG_RED for its own names). Camera is "Webcam 1".

Tuning

Run the Tuning OpMode and pick from the menu. Order that works:

Localization Test, verify pose tracking

Forward/Lateral/Turn Tuners, update ticks-to-inches in Constants

Velocity Tuners, update xVelocity/yVelocity

PIDF tuners, adjust gains
