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
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
@Configurable
public class redddd extends OpMode {
    private DcMotor intake = null;
    private DcMotor takeout1 = null;
    private DcMotor takeout2 = null;


    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer stateTimer = new Timer();
    private IntakeStuff intaker = new IntakeStuff();


    private boolean pathStarted = false;
    private boolean intakeStarted = false;


    private OuttakeStuff outtaker = new OuttakeStuff();
    private boolean outtakeTriggered = false;


    private int pathState;
    private Paths paths;
    private double[] apriltagData = {-1};
    private int detectedID = -1;


    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;


    private static final int STATE_INITIAL_LAUNCH = 0;
    private static final int STATE_RUNNING_TO_GPP = 1;
    private static final int STATE_RUNNING_TO_LAUNCH_1 = 2;
    private static final int STATE_RUNNING_TO_NEXT = 3;
    private static final int STATE_RUNNING_TO_LAUNCH_2 = 4;
    private static final int STATE_TURN = 5;
    private static final int STATE_FINISH = 6;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(76, 9, Math.toRadians(90)));
        follower.setMaxPower(0.65);


        paths = new Paths(follower);


        intake = hardwareMap.dcMotor.get("i");
        takeout1 = hardwareMap.dcMotor.get("f1");
        takeout2 = hardwareMap.dcMotor.get("f2");




        initAprilTag();
        intaker.init(hardwareMap);
        outtaker.init(hardwareMap);


        pathState = STATE_INITIAL_LAUNCH;
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }


    @Override
    public void init_loop() {


        apriltagData = telemetryAprilTag();
        if (apriltagData[0] != -1) {
            detectedID = (int) apriltagData[0];
            panelsTelemetry.debug("Tag Found", detectedID);
        }
        panelsTelemetry.update(telemetry);




    }


    @Override
    public void start() {
        stateTimer.resetTimer();
        pathStarted = false;
        intakeStarted = false;
        if (visionPortal != null) {
            visionPortal.close();
        }
    }


    @Override
    public void loop() {




        panelsTelemetry.update(telemetry);


        follower.update();
        intaker.update();
        outtaker.update();


        pathState = autonomousPathUpdate();


        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Path Started", pathStarted);
        panelsTelemetry.debug("Intake Running", intaker.isBusy());
        panelsTelemetry.debug("Follower Busy", follower.isBusy());
        panelsTelemetry.debug("Timer", stateTimer.getElapsedTime());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }


    public int autonomousPathUpdate() {
        if (detectedID == -1) return pathState;


        switch (pathState) {
            case STATE_INITIAL_LAUNCH:
                if (!pathStarted) {
                    if (detectedID == 21 || detectedID == 22) {
                        follower.followPath(paths.Path1line1);
                    } else if (detectedID == 23) {
                        follower.followPath(paths.Path1line2);
                    }
                    pathStarted = true;
                    stateTimer.resetTimer();
                }


                if (!outtakeTriggered && !follower.isBusy()) {
                    outtaker.startOuttake();
                    outtakeTriggered = true;
                    stateTimer.resetTimer();
                }




                if (outtakeTriggered) {
                    if (stateTimer.getElapsedTime() > 1000) {
                        intaker.startIntake();
                    }
                }




                if (!outtaker.isBusy() && outtakeTriggered) {
                    pathState = STATE_RUNNING_TO_GPP;
                    pathStarted = false;
                    outtakeTriggered = false;
                    intakeStarted = false;
                    stateTimer.resetTimer();


                    intaker.stopIntake();
                }
                break;


            case STATE_RUNNING_TO_GPP:
                if (!pathStarted) {
                    if (detectedID == 21 || detectedID == 22) {
                        follower.followPath(paths.Path2line1);
                    } else if (detectedID == 23) {
                        follower.followPath(paths.Path2line2);
                    }
                    pathStarted = true;
                    stateTimer.resetTimer();
                }


                if (!intakeStarted) {
                    intaker.startIntake();
                    intakeStarted = true;
                }


                if (!follower.isBusy() && stateTimer.getElapsedTime() > 2000) {
                    pathState = STATE_RUNNING_TO_LAUNCH_1;
                    pathStarted = false;
                    intakeStarted = false;
                    stateTimer.resetTimer();
                }
                break;


            case STATE_RUNNING_TO_LAUNCH_1:
                if (!pathStarted) {
                    if (detectedID == 21 || detectedID == 22) {
                        follower.followPath(paths.Path3line1);
                    } else if (detectedID == 23) {
                        follower.followPath(paths.Path3line2);
                    }
                    pathStarted = true;
                    stateTimer.resetTimer();
                }


                if (!outtakeTriggered && !follower.isBusy()) {
                    outtaker.startOuttake();
                    outtakeTriggered = true;
                    stateTimer.resetTimer();
                }




                if (outtakeTriggered) {
                    if (stateTimer.getElapsedTime() > 1000) {
                        intaker.startIntake();
                    }
                }






                if (outtakeTriggered) {
                    if (stateTimer.getElapsedTime() > 1000) {
                        intaker.startIntake();
                    }
                }




                if (!outtaker.isBusy() && outtakeTriggered && stateTimer.getElapsedTime() > 1000) {
                    pathState = STATE_RUNNING_TO_NEXT;
                    pathStarted = false;
                    outtakeTriggered = false;
                    intakeStarted = false;
                    stateTimer.resetTimer();
                    intaker.stopIntake();
                }
                break;


            case STATE_RUNNING_TO_NEXT:
                if (!pathStarted) {
                    if (detectedID == 21 || detectedID == 22) {
                        follower.followPath(paths.Path4line1);
                    } else if (detectedID == 23) {
                        follower.followPath(paths.Path4line2);
                    }
                    pathStarted = true;
                    stateTimer.resetTimer();
                }


                if (!intakeStarted) {
                    intaker.startIntake();
                    intakeStarted = true;
                }


                if (!follower.isBusy() && stateTimer.getElapsedTime() > 2000) {
                    pathState = STATE_RUNNING_TO_LAUNCH_2;
                    pathStarted = false;
                    intakeStarted = false;
                    stateTimer.resetTimer();
                }
                break;


            case STATE_RUNNING_TO_LAUNCH_2:
                if (!pathStarted) {
                    if (detectedID == 21 || detectedID == 22) {
                        follower.followPath(paths.Path5line1);
                    } else if (detectedID == 23) {
                        follower.followPath(paths.Path5line2);
                    }
                    pathStarted = true;
                    stateTimer.resetTimer();
                }


                if (!outtakeTriggered && !follower.isBusy()) {
                    outtaker.startOuttake();
                    outtakeTriggered = true;
                    stateTimer.resetTimer();
                }




                if (outtakeTriggered) {
                    if (stateTimer.getElapsedTime() > 1000) {
                        intaker.startIntake();
                    }
                }


                if (!outtaker.isBusy() && outtakeTriggered && stateTimer.getElapsedTime() > 1000) {
                    pathState = STATE_TURN;
                    pathStarted = false;
                    outtakeTriggered = false;
                    stateTimer.resetTimer();
                }
                break;


            case STATE_TURN:
                if (!pathStarted) {
                    if (detectedID == 21 || detectedID == 22) {
                        follower.followPath(paths.Path6line1);
                    } else if (detectedID == 23) {
                        follower.followPath(paths.Path6line2);
                    }
                    pathStarted = true;
                    stateTimer.resetTimer();
                }


                if (!follower.isBusy() && stateTimer.getElapsedTime() > 500) {
                    pathState = STATE_FINISH;
                }
                break;


            case STATE_FINISH:
                // Autonomous complete
                break;
        }
        return pathState;
    }


    public static class Paths {
        public PathChain Path1line1;
        public PathChain Path2line1;
        public PathChain Path3line1;
        public PathChain Path4line1;
        public PathChain Path5line1;
        public PathChain Path6line1;


        public PathChain Path1line2;
        public PathChain Path2line2;
        public PathChain Path3line2;
        public PathChain Path4line2;
        public PathChain Path5line2;
        public PathChain Path6line2;


        public Paths(Follower follower) {
            Path1line2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(76.000, 9.000),
                                    new Pose(80.000, 20.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(50))
                    .build();


            Path2line2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(80.000, 20.000),
                                    new Pose(84.000, 37.000),
                                    new Pose(132.000, 34.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();


            Path3line2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(132.000, 34.000),
                                    new Pose(129.000, 34.000),
                                    new Pose(72.000, 55.000),
                                    new Pose(72.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                    .build();


            Path4line2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(72.000, 80.000),
                                    new Pose(98.000, 85.000),
                                    new Pose(132.000, 83.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();


            Path5line2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.000, 84.000),
                                    new Pose(72.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                    .build();


            Path6line2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.000, 80.000),
                                    new Pose(72.000, 80.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                    .build();




            Path1line1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(76.000, 9.000),
                                    new Pose(76.000, 13.000),
                                    new Pose(80.000, 20.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))


                    .build();


            Path2line1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(80.000, 20.000),
                                    new Pose(101.628, 38.897),
                                    new Pose(133.159, 35.159)
                            )
                    ).setTangentHeadingInterpolation()


                    .build();


            Path3line1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(133.159, 35.159),
                                    new Pose(129.000, 34.000),
                                    new Pose(72.000, 55.000),
                                    new Pose(72.000, 90.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))


                    .build();


            Path4line1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(72.000, 90.000),
                                    new Pose(92.386, 68.434),
                                    new Pose(132.662, 75.717)
                            )
                    ).setTangentHeadingInterpolation()


                    .build();


            Path5line1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(132.662, 75.717),
                                    new Pose(83.779, 58.007),
                                    new Pose(72.000, 90.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))


                    .build();


            Path6line1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(72.000, 90.000),
                                    new Pose(72.000, 80.000),
                                    new Pose(72.000, 72.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))


                    .build();
        }
    }


    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }


    private double[] telemetryAprilTag() {
        double ID = -1;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();


        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                ID = detection.id;
                break;
            }
        }
        return new double[]{ID};
    }


    public void sleep(long milliseconds) {
        long pauseUntil = (long) getRuntime() + milliseconds;
        while (getRuntime() < pauseUntil) { }
    }
}



