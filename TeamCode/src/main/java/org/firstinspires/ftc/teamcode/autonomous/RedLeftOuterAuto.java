package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.teamcode.util.Constants.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.Constants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_ARM_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_ARM_OUT_POSITION;
//import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.Constants.LIFT_TRANSFER_UPPER_LIMIT;
//import static org.firstinspires.ftc.teamcode.util.Constants.OUTER_OUTTAKE_CLAW_CLOSED;
//import static org.firstinspires.ftc.teamcode.util.Constants.OUTER_OUTTAKE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_ARM_YELLOW_SCORE_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_CLAW_DROP_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.Constants.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.util.Constants.ROBOT_FRONT_LENGTH;
import static org.firstinspires.ftc.teamcode.util.Constants.TRANSFER_OUT;
import static org.firstinspires.ftc.teamcode.util.Constants.TRANSFER_RESET;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.TwoPersonDrive;
import org.firstinspires.ftc.teamcode.util.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.util.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.util.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.util.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.util.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.util.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.util.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.util.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.pedroPathing.util.Timer;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Far Side 2 + 0", group = "Autonomous")
public class RedLeftOuterAuto extends OpMode {

    private TwoPersonDrive twoPersonDrive;

    private Timer pathTimer, opmodeTimer, scanTimer, distanceSensorDecimationTimer;

    private VisionPortal visionPortal;

    private String navigation;

    private DistanceSensor rearDistanceSensor;

    private boolean rearDistanceSensorDisconnected;

    // IMPORTANT: y increasing is towards the backstage from the audience,
    // while x increasing is towards the red side from the blue side
    // this means that 0 heading is pointing from the blue side to the red side

    // all spike mark locations since I'm lazy
    private Pose redLeftSideLeftSpikeMark = new Pose(36 + 72, -47.5 + 72);
    private Pose redLeftSideMiddleSpikeMark = new Pose(24.5 + 72, -36 + 72);
    private Pose redLeftSideRightSpikeMark = new Pose(36 + 72, -24.5 + 72);
    private Pose redRightSideLeftSpikeMark = new Pose(36 + 72, 0.5 + 72);
    private Pose redRightSideMiddleSpikeMark = new Pose(24.5 + 72, 12 + 72);
    private Pose redRightSideRightSpikeMark = new Pose(36 + 72, 23.5 + 72);
    private Pose blueLeftSideLeftSpikeMark = new Pose(-36 + 72, 23.5 + 72);
    private Pose blueLeftSideMiddleSpikeMark = new Pose(-24.5 + 72, 12 + 72);
    private Pose blueLeftSideRightSpikeMark = new Pose(-36 + 72, 0.5 + 72);
    private Pose blueRightSideLeftSpikeMark = new Pose(-36 + 72, -24.5 + 72);
    private Pose blueRightSideMiddleSpikeMark = new Pose(-24.5 + 72, -36 + 72);
    private Pose blueRightSideRightSpikeMark = new Pose(-36 + 72, -47.5 + 72);

    // backdrop april tag locations
    private Pose blueLeftBackdrop = new Pose(-42.875 + 72, 60.75 + 72);
    private Pose blueMiddleBackdrop = new Pose(-36.75 + 72, 60.75 + 72);
    private Pose blueRightBackdrop = new Pose(-30.75 + 72, 60.75 + 72);
    private Pose redLeftBackdrop = new Pose(30.75 + 72, 60.75 + 72);
    private Pose redMiddleBackdrop = new Pose(36.75 + 72, 60.75 + 72);
    private Pose redRightBackdrop = new Pose(42.875 + 72, 60.75 + 72);

    // white pixel stack locations
    private Pose redOuterStack = new Pose(36 + 72, -72 + 72);
    private Pose redMiddleStack = new Pose(24 + 72, -72 + 72);
    private Pose redInnerStack = new Pose(12 + 72, -72 + 72);
    private Pose blueInnerStack = new Pose(-12 + 72, -72 + 72);
    private Pose blueMiddleStack = new Pose(-24 + 72, -72 + 72);
    private Pose blueOuterStack = new Pose(-36 + 72, -72 + 72);

    private Pose spikeMarkGoalPose, initialBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose startPose = new Pose(63 + 72, 36, Math.PI);

    // TODO: dont forget to adjust this too
    private Point abortPoint = new Point(135, 120, Point.CARTESIAN), backdropGoalPoint;

    private Follower follower;

    private Path scoreSpikeMark, adjustHeadingFromSpikeMark;
    private PathChain initialScoreOnBackdrop;

    private int pathState;

    public void setBackdropGoalPose() {
        switch (navigation) {
            default:
            case "left":
                spikeMarkGoalPose = new Pose(redLeftSideLeftSpikeMark.getX() - 2, redLeftSideLeftSpikeMark.getY() + 3, Math.PI / 2);
                initialBackdropGoalPose = new Pose(redLeftBackdrop.getX() + 4, redLeftBackdrop.getY() - ROBOT_BACK_LENGTH + 0.25, Math.PI * 1.5);
                break;
            case "middle":
                spikeMarkGoalPose = new Pose(redLeftSideMiddleSpikeMark.getX(), redLeftSideMiddleSpikeMark.getY() - 4, Math.PI / 2);
                initialBackdropGoalPose = new Pose(redMiddleBackdrop.getX() + 5.75, redMiddleBackdrop.getY() - ROBOT_BACK_LENGTH, Math.PI * 1.5);
                break;
            case "right":
                spikeMarkGoalPose = new Pose(redLeftSideRightSpikeMark.getX() - 2, redLeftSideRightSpikeMark.getY() + 1, Math.PI / 2);
                initialBackdropGoalPose = new Pose(redRightBackdrop.getX() + 5, redRightBackdrop.getY() - ROBOT_BACK_LENGTH + 0.25, Math.PI * 1.5);
                break;
        }
    }

    public void buildPaths() {
        Point scoreSpikeMarkMidPoint;
        double scoreSpikeMarkMidToSpikeDistance;
        switch (navigation) {
            default:
            case "left":
                scoreSpikeMarkMidPoint = new Point(startPose.getX() - 12.5, startPose.getY() - 15, Point.CARTESIAN);
                break;
            case "middle":
                scoreSpikeMarkMidPoint = new Point(startPose.getX() - 5.5, startPose.getY() - 22, Point.CARTESIAN);
                break;
            case "right":
                scoreSpikeMarkMidPoint = new Point(startPose.getX() - 3.5, startPose.getY() - 7, Point.CARTESIAN);
                break;
        }
        scoreSpikeMarkMidToSpikeDistance = MathFunctions.distance(spikeMarkGoalPose, scoreSpikeMarkMidPoint);
        scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), scoreSpikeMarkMidPoint, new Point(spikeMarkGoalPose.getX() + MathFunctions.getSign(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * Math.abs(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, spikeMarkGoalPose.getY() + MathFunctions.getSign(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * Math.abs(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * ROBOT_FRONT_LENGTH / scoreSpikeMarkMidToSpikeDistance, Point.CARTESIAN)));
        scoreSpikeMark.setConstantHeadingInterpolation(startPose.getHeading());
        scoreSpikeMark.setPathEndTimeoutConstraint(3);

        adjustHeadingFromSpikeMark = new Path(new BezierLine(scoreSpikeMark.getLastControlPoint(), new Point(132, 30, Point.CARTESIAN)));
        adjustHeadingFromSpikeMark.setLinearHeadingInterpolation(scoreSpikeMark.getEndTangent().getTheta(), Math.PI * 1.5);

        initialScoreOnBackdrop = follower.pathBuilder()
                .addPath(new BezierLine(adjustHeadingFromSpikeMark.getLastControlPoint(), new Point(135, 85, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .addPath(new BezierCurve(new Point(135, 85, Point.CARTESIAN), new Point(128, 105, Point.CARTESIAN), new Point(initialBackdropGoalPose)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .setPathEndTimeoutConstraint(2.5)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10: // starts following the first path to score on the spike mark
                follower.followPath(scoreSpikeMark);
                setPathState(11);
                break;
            case 11: // detects the path to progress away from the wall and sets tangent interpolation
                if (follower.getCurrentTValue() > 0.1) {
                    //scoreSpikeMark.setReversed(false);
                    scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading() - 0.1 * MathFunctions.getTurnDirection(startPose.getHeading(), scoreSpikeMark.getEndTangent().getTheta()) * MathFunctions.getSmallestAngleDifference(startPose.getHeading(), scoreSpikeMark.getEndTangent().getTheta()), scoreSpikeMark.getEndTangent().getTheta());
                    setPathState(12);
                }
                break;
            case 12: // detects for the end of the path and everything else to be in order and releases the pixel
                if (!follower.isBusy() && twoPersonDrive.intakeState == INTAKE_OUT) {
//                    twoPersonDrive.setIntakeClawOpen(true);
                    setPathState(13);
                }
                break;
//            case 13: // moves mechanisms into position to score and pick up from stack as well as starts moving to score
//                if (pathTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME) {
//                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
//                    setPathState(14);
//                }
//                break;
            case 14:
                if (pathTimer.getElapsedTime() > 500) {
                    follower.followPath(adjustHeadingFromSpikeMark);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(initialScoreOnBackdrop);
                    setPathState(16);
                }
                break;
            case 17: // detects for end of the path and outtake out and drops pixel
                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    Follower.useHeading = false;
                    backdropGoalPoint = new Point(initialBackdropGoalPose);
                    follower.holdPoint(new BezierPoint(backdropGoalPoint), Math.PI * 1.5);
                    twoPersonDrive.setV4BInterpolation(OUTTAKE_ARM_YELLOW_SCORE_POSITION);
                    distanceSensorDecimationTimer.resetTimer();
                    setPathState(18);
                }
                break;

            case 19:
//                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
//                    setPathState(110);
//                }
                break;
            case 22:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                requestOpModeStop();
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    @Override
    public void loop() {
        follower.update();
        twoPersonDrive.autonomousControlUpdate();

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        twoPersonDrive.telemetry();
        //telemetry.update();
    }

    @Override
    public void init() {
        //PhotonCore.start(this.hardwareMap);

        twoPersonDrive = new TwoPersonDrive(true);
        twoPersonDrive.hardwareMap = hardwareMap;
        twoPersonDrive.telemetry = telemetry;

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        scanTimer = new Timer();
        distanceSensorDecimationTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        twoPersonDrive.initialize();

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        twoPersonDrive.claw.setPosition(CLAW_CLOSED);

        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        try {
            sleep(2500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        scanTimer.resetTimer();
    }

    @Override
    public void start() {
        setBackdropGoalPose(); //TODO: please change
        buildPaths();
        twoPersonDrive.frameTimer.resetTimer();
        opmodeTimer.resetTimer();
        setPathState(10);
    }

    @Override
    public void stop() {
    }
}

/**
 * 8==D
 */