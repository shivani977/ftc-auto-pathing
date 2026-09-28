package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
@Configurable
public class BIG_RED extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private final Timer stateTimer = new Timer();

    private final IntakeStuff intaker = new IntakeStuff();
    private final OuttakeStuff outtaker = new OuttakeStuff();

    private boolean intakeTriggered = false;
    private boolean outtakeTriggered = false;

    private PathState currentState;
    private Paths paths;
    private int detectedID = -1;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private enum PathState {
        STATE_LAUNCH,
        STATE_START,
        STATE_RUNNING_PATH_1,
        STATE_OUTTAKE_SEQUENCE,
        STATE_FINISH
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(122, 123, Math.toRadians(180 - 143)));
        follower.setMaxPower(0.8);

        paths = new Paths(follower);

        initAprilTag();
        intaker.init(hardwareMap);
        outtaker.init(hardwareMap);
        outtaker.flywheel_power = 1.0;

        currentState = PathState.STATE_LAUNCH;
        stateTimer.resetTimer();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        intaker.update();
        outtaker.update();
        updateAprilTagDetection();

        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", currentState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Detected AprilTag", detectedID);
        panelsTelemetry.update(telemetry);
    }

    private void autonomousPathUpdate() {
        switch (currentState) {
            case STATE_LAUNCH:
                if (stateTimer.getElapsedTime() > 200 && !follower.isBusy()) {
                    if (paths.Path0line1 != null) {
                        follower.followPath(paths.Path0line1);
                    }

                    if (!outtakeTriggered) {
                        outtaker.startOuttake();
                        outtakeTriggered = true;
                    }
                }

                if (!intakeTriggered && stateTimer.getElapsedTime() > 2000) {
                    intaker.startIntake();
                    intakeTriggered = true;
                }

                if (intakeTriggered && stateTimer.getElapsedTime() > 7000) {
                    setPathState(PathState.STATE_FINISH);
                }
                break;

            case STATE_START:
                if (stateTimer.getElapsedTime() > 200 && !follower.isBusy()) {
                    intaker.startIntake();
                    intakeTriggered = true;

                    if (paths.Path1line1 != null) {
                        follower.followPath(paths.Path1line1);
                    }
                    setPathState(PathState.STATE_RUNNING_PATH_1);
                }
                break;

            case STATE_RUNNING_PATH_1:
                if (stateTimer.getElapsedTime() > 200 && !follower.isBusy()) {
                    if (paths.Path2line1 != null) {
                        follower.followPath(paths.Path2line1);
                    }
                    intakeTriggered = false;
                    outtakeTriggered = false;
                    setPathState(PathState.STATE_OUTTAKE_SEQUENCE);
                }
                break;

            case STATE_OUTTAKE_SEQUENCE:
                outtaker.flywheel_power = 0.95;
                if (stateTimer.getElapsedTime() > 200 && !follower.isBusy()) {
                    if (!outtakeTriggered) {
                        outtaker.startOuttake();
                        outtakeTriggered = true;
                        stateTimer.resetTimer();
                    }

                    if (outtakeTriggered && !intakeTriggered && stateTimer.getElapsedTime() > 1000) {
                        intaker.startIntake();
                        intakeTriggered = true;
                        setPathState(PathState.STATE_FINISH);
                    }
                }
                break;

            case STATE_FINISH:
                break;
        }
    }

    private void setPathState(PathState newState) {
        currentState = newState;
        stateTimer.resetTimer();
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    private void updateAprilTagDetection() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                detectedID = detection.id;
                return;
            }
        }
        detectedID = -1;
    }

    public static class Paths {
        public PathChain Path0line1, Path1line1, Path2line1;

        public Paths(Follower follower) {
            Path0line1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(122.000, 123.000),
                                    new Pose(72.000, 72.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180 - 143), Math.toRadians(180 - 135))
                    .build();

            Path1line1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(72.000, 72.000),
                                    new Pose(48.000, 30.000),
                                    new Pose(15.000, 36.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Path2line1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.000, 36.000),
                                    new Pose(72.000, 72.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();
        }
    }
}