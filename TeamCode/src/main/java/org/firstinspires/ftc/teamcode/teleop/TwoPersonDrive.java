package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.util.Constants.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.util.Constants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_ARM_AVOID_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_ARM_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_AVOID;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.Constants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.Constants.LIFT_HIGH_PRESET_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.LIFT_LOW_PRESET_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.LIFT_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.MAX_OUTTAKE_ARM_OUT_IN_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_ARM_DEGREES_TO_SERVO;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_ARM_DROP_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_ARM_FINE_ADJUST_LOWER_BOUND;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_ARM_FINE_ADJUST_UPPER_BOUND;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_ARM_IN_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.MIN_OUTTAKE_ARM_OUT_IN_TIME;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_ARM_OUT_POSITION;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_ARM_PRESET_SPEED;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_ARM_SERVO_TO_DEGREES;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_GOING_IN;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_GOING_OUT;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_IN;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_OUT;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_WAIT;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_WRIST_DEGREES_TO_SERVO;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_WRIST_FINE_ADJUST_DEGREES_PER_SECOND;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_WRIST_FINE_ADJUST_LOWER_BOUND;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_WRIST_FINE_ADJUST_UPPER_BOUND;
import static org.firstinspires.ftc.teamcode.util.Constants.OUTTAKE_WRIST_VERTICAL_OFFSET;
import static org.firstinspires.ftc.teamcode.util.Constants.RIGHT_OUTTAKE_ARM_OFFSET;
import static org.firstinspires.ftc.teamcode.util.Constants.TRANSFER_IDLE;
import static org.firstinspires.ftc.teamcode.util.Constants.TRANSFER_OUT;
import static org.firstinspires.ftc.teamcode.util.Constants.TRANSFER_POSITIONING;
import static org.firstinspires.ftc.teamcode.util.Constants.TRANSFER_PRESET_HOLD;
import static org.firstinspires.ftc.teamcode.util.Constants.TRANSFER_RESET;
import static org.firstinspires.ftc.teamcode.util.Constants.liftPIDFCoefficients;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.util.pedroPathing.util.NanoTimer;
import org.firstinspires.ftc.teamcode.util.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.pedroPathing.util.SingleRunAction;
import org.firstinspires.ftc.teamcode.util.pedroPathing.util.Timer;

@TeleOp(name = "Two Person Drive", group = "Drive")
public class TwoPersonDrive extends LinearOpMode {

    public DcMotorEx leftFront, leftRear, rightFront, rightRear, leftLift, rightLift, liftEncoder, intake;
    public Servo leftV4B, rightV4B, pitch, claw, leftExtendo, rightExtendo, wrist, door;
    public PIDFController liftPIDF;

    public SingleRunAction clawClosed, clawOpen, V4BIn, V4BOut, V4BWait, liftManualControlReset, startTransfer, highPreset, lowPreset, resetPreset, intakeClawOpen, innerOuttakeClawClose, outerOuttakeClawClose, transferPresetHold, putOuttakeOut, outtakeClawsOpen, transferReset, intakeReset, intakeOut, clawToggle, toggleV4BPWM, dropV4B;
    public Timer outtakeTimer, transferTimer, pickUpAdjustTimer;
    public NanoTimer frameTimer;
    public Telemetry telemetryA;
    public VoltageSensor controlHubVoltageSensor;

    public boolean autonomous = false, turnOffV4BPWM;
    public long outtakeMovementTime;
    public double deltaTimeSeconds, outtakeWristDirection, pitchOffset, V4BTargetPosition, outtakePreviousStaticPosition;
    public int intakeState, outtakeState, V4BTargetState, liftTargetPosition, liftPresetTargetPosition, transferState, intakeSpeed, outtakeSpeed;

    public TwoPersonDrive() {
    }

    public TwoPersonDrive(boolean setAuto) {
        autonomous = setAuto;
    }

    public void initialize() {
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        rightRear = hardwareMap.get(DcMotorEx.class, "br");
        rightFront = hardwareMap.get(DcMotorEx.class, "lr");
        leftLift = hardwareMap.get(DcMotorEx.class, "lift");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftExtendo = hardwareMap.get(Servo.class, "leftExtendo");
        rightExtendo = hardwareMap.get(Servo.class, "rightExtendo");
        leftV4B = hardwareMap.get(Servo.class, "leftV4B");
        rightV4B = hardwareMap.get(Servo.class, "rightV4B");
        pitch = hardwareMap.get(Servo.class, "pitch");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        outtakeTimer = new Timer();
        frameTimer = new NanoTimer();
        pickUpAdjustTimer = new Timer();
        transferTimer = new Timer();

        liftPIDF = new PIDFController(liftPIDFCoefficients);
        liftTargetPosition = 0;
        liftPresetTargetPosition = 0;
        liftManualControlReset = new SingleRunAction(() -> setLiftTargetPosition(liftEncoder.getCurrentPosition()));

        V4BIn = new SingleRunAction(() -> setV4BInterpolation(OUTTAKE_ARM_IN_POSITION));
        V4BOut = new SingleRunAction(() -> setV4BInterpolation(OUTTAKE_ARM_OUT_POSITION));
        V4BWait = new SingleRunAction(() -> {
            if (!MathFunctions.roughlyEquals(leftV4B.getPosition(), OUTTAKE_ARM_DROP_POSITION)) {
                setV4BInterpolation(outtakePreviousStaticPosition);
            }
        });
        startTransfer = new SingleRunAction(() -> {
            setLiftTargetPosition(0);
            moveOuttake(OUTTAKE_IN);
            claw.setPosition(CLAW_OPEN);
        });
        clawClosed = new SingleRunAction(() ->  claw.setPosition(CLAW_CLOSED));
        clawOpen = new SingleRunAction(() -> claw.setPosition(CLAW_OPEN));
        transferPresetHold = new SingleRunAction(() -> {
            liftPresetTargetPosition = 0;
            setLiftTargetPosition(liftPresetTargetPosition);
        });
        putOuttakeOut = new SingleRunAction(() -> moveOuttake(OUTTAKE_OUT));
        transferReset = new SingleRunAction(() -> {
            moveOuttake(OUTTAKE_IN);
            setLiftTargetPosition(0);
        });
        highPreset = new SingleRunAction(() -> {
            liftPresetTargetPosition = LIFT_HIGH_PRESET_POSITION;
        });
        lowPreset = new SingleRunAction(() -> {
            liftPresetTargetPosition = LIFT_LOW_PRESET_POSITION;
        });
        resetPreset = new SingleRunAction(() -> {
        });
        clawToggle = new SingleRunAction(() -> {
            if (MathFunctions.roughlyEquals( claw.getPosition(), CLAW_CLOSED)) {
                 claw.setPosition(CLAW_OPEN);
            } else {
                 claw.setPosition(CLAW_CLOSED);
            }
        });
        toggleV4BPWM = new SingleRunAction(()-> {
            if (turnOffV4BPWM) {
                turnOffV4BPWM = false;
                leftV4B.getController().pwmEnable();
                rightV4B.getController().pwmEnable();
            } else {
                turnOffV4BPWM = true;
            }
        });
        dropV4B = new SingleRunAction(()-> {
            setV4BInterpolation(OUTTAKE_ARM_DROP_POSITION);
            setOuttakeWristDirection(-5);
        });
        claw.setPosition(CLAW_OPEN);
        transferState = TRANSFER_IDLE;

        setEncoderMotors();

        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        sleep(1000);

        leftV4B.setPosition(OUTTAKE_ARM_IN_POSITION);
        rightV4B.setPosition(OUTTAKE_ARM_IN_POSITION + RIGHT_OUTTAKE_ARM_OFFSET);
        setOuttakeWristDirection(-15);

        outtakeState = OUTTAKE_IN;
        V4BTargetState = OUTTAKE_IN;
        V4BTargetPosition = leftV4B.getPosition();
        setV4BInterpolation(V4BTargetPosition);
        outtakePreviousStaticPosition = OUTTAKE_ARM_IN_POSITION;
    }

    public void setEncoderMotors() {
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftEncoder = rightLift;
    }

    @Override
    public void runOpMode() {
        work();
    }

    public void work() {
        while (!isStarted() && !isStopRequested()) {
        }

        initialize();

        frameTimer.resetTimer();
        leftV4B.getController().pwmEnable();
        rightV4B.getController().pwmEnable();


        while (opModeIsActive()) {
            driverControlUpdate();

            telemetry();
        }
    }

    public void telemetry() {
        telemetryA.addData("Intake State", intakeState);
        telemetryA.addData("Outtake State", outtakeState);
//        telemetryA.addData("Transfer State", transferState);
        telemetryA.addData("Lift Position", liftEncoder.getCurrentPosition());
        telemetryA.addData("Lift Target Position", liftTargetPosition);
        telemetryA.addData("Lift Current", liftEncoder.getCurrent(CurrentUnit.MILLIAMPS));
        telemetryA.addData("Pitch Offset", pitchOffset);
        telemetryA.update();
    }

    public void driverControlUpdate() {
        updateFrameTime();

        drive();

        buttonControls();

        teleopLiftControlUpdate();

        updateServoMechanisms();

        fineAdjustControls();
    }

    public void autonomousControlUpdate() {
        updateFrameTime();

        updateLift();

        updateServoMechanisms();
    }

    public void drive() {
        double throttle = 0.4 + 0.6 * gamepad1.right_trigger;


        double y = -gamepad1.left_stick_y * throttle; // Remember, this is reversed!

        double x = 0; // this is strafing
        if (gamepad1.left_bumper) {
            x -= 1;
        }
        if (gamepad1.right_bumper) {
            x += 1;
        }
        x *= throttle;

        double rx = 0;
        if (Math.abs(gamepad1.left_stick_x) > 0.1) {
            if (MathFunctions.roughlyEquals(0.4, throttle)) {
                rx = gamepad1.left_stick_x * throttle * 0.7;
            } else {
                rx = gamepad1.left_stick_x * throttle;
            }
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        if (controlHubVoltageSensor.getVoltage() < 8.5) {
            denominator *= 2;
        }

        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    public void buttonControls() {

        // intake presets
        if (gamepad1.circle || gamepad1.b) {
            intakeReset.run();
        } else {
            intakeReset.reset();
        }
        if (gamepad1.a || gamepad1.cross) {
            intakeOut.run();
        } else {
            intakeOut.reset();
        }

        // presets
        if (!turnOffV4BPWM) {
            if (gamepad2.dpad_up) {
                highPreset.run();
            } else {
                highPreset.reset();
            }
            if (gamepad2.dpad_down) {
                lowPreset.run();
            } else {
                lowPreset.reset();
            }
            if (gamepad2.dpad_right) {
                resetPreset.run();
            } else {
                resetPreset.reset();
            }
        }

        /*
        // toggle outtake arm on/off for hang
        if (gamepad2.a || gamepad2.cross) {
            toggleV4BPWM.run();
        } else {
            toggleV4BPWM.reset();
        }
         */
    }

    public void teleopLiftControlUpdate() {
        if ((outtakeState == OUTTAKE_OUT && transferState == TRANSFER_IDLE) && (Math.abs(gamepad2.left_stick_y) > 0) && ((liftEncoder.getCurrentPosition() < LIFT_MAX_POSITION && liftEncoder.getCurrentPosition() > 0) || (liftEncoder.getCurrentPosition() >= LIFT_MAX_POSITION && -gamepad2.left_stick_y < 0) || (liftEncoder.getCurrentPosition() <= 0 && -gamepad2.left_stick_y > 0))) {
            leftLift.setPower(-gamepad2.left_stick_y);
            rightLift.setPower(-gamepad2.left_stick_y);
            liftManualControlReset.reset();
        } else {
            liftManualControlReset.run();
            updateLift();
        }
    }

    public void updateServoMechanisms() {
        updateOuttake();
        updatePitch();
    }

    public void fineAdjustControls() {
        adjustV4B();
        adjustPitch();
    }

    public void adjustV4B() {
        if (outtakeState == OUTTAKE_OUT) {
            if (leftV4B.getPosition() >= OUTTAKE_ARM_FINE_ADJUST_LOWER_BOUND && leftV4B.getPosition() <= OUTTAKE_ARM_FINE_ADJUST_UPPER_BOUND) {
                setV4BInterpolation(V4BTargetPosition + gamepad2.right_stick_y * deltaTimeSeconds * OUTTAKE_ARM_FINE_ADJUST_DEGREES_PER_SECOND * OUTTAKE_ARM_DEGREES_TO_SERVO);
            } else {
                if (leftV4B.getPosition() > OUTTAKE_ARM_FINE_ADJUST_UPPER_BOUND) {
                    setV4BInterpolation(OUTTAKE_ARM_FINE_ADJUST_UPPER_BOUND);
                } else if (leftV4B.getPosition() < OUTTAKE_ARM_FINE_ADJUST_LOWER_BOUND) {
                    setV4BInterpolation(OUTTAKE_ARM_FINE_ADJUST_LOWER_BOUND);
                }
            }
        }
    }

    public void adjustPitch() {
        if (outtakeState == OUTTAKE_OUT) {
            if (pitchOffset <= OUTTAKE_WRIST_FINE_ADJUST_LOWER_BOUND && pitchOffset >= OUTTAKE_WRIST_FINE_ADJUST_UPPER_BOUND) {
                pitchOffset += (gamepad2.right_trigger - gamepad2.left_trigger) * deltaTimeSeconds * OUTTAKE_WRIST_FINE_ADJUST_DEGREES_PER_SECOND;
            } else {
                if (pitchOffset < OUTTAKE_WRIST_FINE_ADJUST_UPPER_BOUND) {
                    pitchOffset = OUTTAKE_WRIST_FINE_ADJUST_UPPER_BOUND;
                } else if (pitchOffset > OUTTAKE_WRIST_FINE_ADJUST_LOWER_BOUND) {
                    pitchOffset = OUTTAKE_WRIST_FINE_ADJUST_LOWER_BOUND;
                }
            }
        }
    }

    public void resetTransferActions() {
        startTransfer.reset();
        intakeClawOpen.reset();
        innerOuttakeClawClose.reset();
        outerOuttakeClawClose.reset();
        transferPresetHold.reset();
        putOuttakeOut.reset();
        outtakeClawsOpen.reset();
        transferReset.reset();
        dropV4B.reset();
    }

    public void resetAllActions() {
        resetOuttakeActions();
        resetTransferActions();
    }

    public void resetOuttakeActions() {
        V4BIn.reset();
        V4BOut.reset();
        V4BWait.reset();
    }

    public void moveOuttake(int state) {
        switch (state) {
            case OUTTAKE_IN:
            case OUTTAKE_OUT:
                V4BTargetState = state;
                resetOuttakeActions();
                updateOuttake();
                break;
        }
    }

    public void setOuttakeState(int state) {
        switch (state) {
            case OUTTAKE_IN:
            case OUTTAKE_OUT:
            case OUTTAKE_GOING_IN:
            case OUTTAKE_GOING_OUT:
                outtakeTimer.resetTimer();
                outtakeState = state;
                resetOuttakeActions();
                break;
            case OUTTAKE_WAIT:
                V4BWait.run();
                outtakeTimer.resetTimer();
                outtakeState = state;
                resetOuttakeActions();
                break;
        }
        updateOuttake();
    }

    public void updateOuttake() {
        if (!(outtakeState == OUTTAKE_WAIT)) {
            switch (V4BTargetState) {
                case OUTTAKE_IN:
                    switch (outtakeState) {
                        case OUTTAKE_IN:
                            if (transferState == TRANSFER_IDLE) {
                                setOuttakeWristDirection(-7);
                                pitchOffset = 0;
                            }
                            outtakePreviousStaticPosition = OUTTAKE_ARM_IN_POSITION;
                            break;
                        case OUTTAKE_OUT:
                            setOuttakeWristDirection(120);
                            outtakePreviousStaticPosition = OUTTAKE_ARM_OUT_POSITION;
                            outtakeMovementTime = getVariableOuttakeMoveTime();
                            setOuttakeState(OUTTAKE_GOING_IN);
                            break;
                        case OUTTAKE_GOING_IN:
                            setOuttakeWristDirection(0);
                            pitchOffset = 0;
                            V4BIn.run();
                            if (outtakeTimer.getElapsedTime() > outtakeMovementTime && V4BAtTargetPosition()) {
                                setOuttakeState(OUTTAKE_IN);
                            }
                            break;
                        case OUTTAKE_GOING_OUT:
                            setOuttakeWristDirection(120);
                            outtakeMovementTime = getVariableOuttakeMoveTime();;
                            setOuttakeState(OUTTAKE_GOING_IN);
                            break;
                    }
                    break;
                case OUTTAKE_OUT:
                    switch (outtakeState) {
                        case OUTTAKE_IN:
                            setOuttakeWristDirection(-7);
                            pitchOffset = 0;
                            outtakePreviousStaticPosition = OUTTAKE_ARM_IN_POSITION;
                            outtakeMovementTime = MIN_OUTTAKE_ARM_OUT_IN_TIME;
                            setOuttakeState(OUTTAKE_GOING_OUT);
                            break;
                        case OUTTAKE_OUT:
                            setOuttakeWristDirection(120);
                            outtakePreviousStaticPosition = OUTTAKE_ARM_OUT_POSITION;
                            break;
                        case OUTTAKE_GOING_IN:
                            setOuttakeWristDirection(0);
                            pitchOffset = 0;
                            outtakeMovementTime = MIN_OUTTAKE_ARM_OUT_IN_TIME;
                            setOuttakeState(OUTTAKE_GOING_OUT);
                            break;
                        case OUTTAKE_GOING_OUT:
                            setOuttakeWristDirection(120);
                            V4BOut.run();
                            if (outtakeTimer.getElapsedTime() > outtakeMovementTime && V4BAtTargetPosition()) {
                                setOuttakeState(OUTTAKE_OUT);
                            }
                            break;
                    }
                    break;
            }
        }
        if (turnOffV4BPWM && (outtakeState == OUTTAKE_IN || outtakeState == OUTTAKE_OUT)) {
            leftV4B.getController().pwmDisable();
            rightV4B.getController().pwmDisable();
        }
        updateV4BInterpolation();
    }

    public long getVariableOuttakeMoveTime() {
        return (long)(MIN_OUTTAKE_ARM_OUT_IN_TIME + (MathFunctions.clamp((OUTTAKE_ARM_FINE_ADJUST_UPPER_BOUND - leftV4B.getPosition()) / (OUTTAKE_ARM_FINE_ADJUST_UPPER_BOUND - OUTTAKE_ARM_FINE_ADJUST_LOWER_BOUND), 0,1) * (MAX_OUTTAKE_ARM_OUT_IN_TIME - MIN_OUTTAKE_ARM_OUT_IN_TIME)));
    }

    public void updateLift() {
        liftPIDF.updateError(liftTargetPosition - liftEncoder.getCurrentPosition());
        double liftPower = liftPIDF.runPIDF();
        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);
    }

    public void setLiftTargetPosition(int position) {
        liftTargetPosition = position;
        liftPIDF.reset();
        updateLift();
    }

    public void setV4BPosition(double position) {
        if (!MathFunctions.roughlyEquals(leftV4B.getPosition(), position)) {
            leftV4B.setPosition(position);
            //rightV4B.setPosition(1 - position + RIGHT_OUTTAKE_ARM_OFFSET);
            rightV4B.setPosition(position + RIGHT_OUTTAKE_ARM_OFFSET);
            updatePitch();
        }
    }

    public void setV4BInterpolation(double targetPosition, int speed) {
        V4BTargetPosition = targetPosition;
        outtakeSpeed = speed;
        updateV4BInterpolation();
    }

    public void setV4BInterpolation(double targetPosition) {
        setV4BInterpolation(targetPosition, OUTTAKE_ARM_PRESET_SPEED);
    }

    public void updateV4BInterpolation() {
        double direction = MathFunctions.getSign(V4BTargetPosition - leftV4B.getPosition());
        double nextPosition = leftV4B.getPosition() + direction * (double)outtakeSpeed * deltaTimeSeconds * OUTTAKE_ARM_DEGREES_TO_SERVO;
        if ((direction > 0 && nextPosition > V4BTargetPosition) || (direction < 0 && nextPosition < V4BTargetPosition)) nextPosition = V4BTargetPosition;
        setV4BPosition(nextPosition);
    }

    public boolean V4BAtTargetPosition() {
        return MathFunctions.roughlyEquals(leftV4B.getPosition(), V4BTargetPosition);
    }

    /**
     * Sets what direction we want the outtake wrist to point. This will be an absolute direction,
     * so the outtake arm's current angle should not influence the direction the wrist points.
     * 0 is pointing directly upwards and increasing the direction goes out of the robot
     *
     * @param direction the direction the outtake wrist will point. This is an absolute direction
     *                  expressed in degrees
     */
    public void setOuttakeWristDirection(double direction) {
        outtakeWristDirection = direction;
        updatePitch();
    }

    /**
     * Updates the wrist's position so it keeps its angle as the outtake arm moves
     */
    public void updatePitch() {
        double position = (outtakeWristDirection + pitchOffset) * OUTTAKE_WRIST_DEGREES_TO_SERVO + OUTTAKE_WRIST_VERTICAL_OFFSET - (OUTTAKE_ARM_IN_POSITION - leftV4B.getPosition()) * OUTTAKE_ARM_SERVO_TO_DEGREES * OUTTAKE_WRIST_DEGREES_TO_SERVO;
        if (!MathFunctions.roughlyEquals(pitch.getPosition(), position))
            pitch.setPosition(position);
    }

    public void updateFrameTime() {
        deltaTimeSeconds = frameTimer.getElapsedTimeSeconds();
        frameTimer.resetTimer();
    }
    
}