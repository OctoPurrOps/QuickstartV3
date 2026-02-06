package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BLUE-LONG", group = "Autonomous")
public class blue_auto2 extends OpMode {

    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    // TOTAL shooter speeds (split across two motors)
    private static final double SHOOTER_TOTAL_BASE = 2620;   // 800 each
    private static final double SHOOTER_TOTAL_TARGET = 3100; // 900 each

    private Servo shooterServo;
    private static final double SHOOTER_SERVO_START_POS = 0.03;

    private DcMotorEx intake;
    private static final double INTAKE_POWER = -1.0;

    private DcMotor aimMotor;

    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;
    private static final double HEADING_OFFSET = Math.toRadians(0);
    private static final double TICKS_PER_REV = 1627;
    private static final double AIM_TICKS_PER_RAD = TICKS_PER_REV / (2.0 * Math.PI);
    private static final int AIM_MIN_TICKS = -2000;
    private static final int AIM_MAX_TICKS = 2000;
    private static final double AIM_POWER = 1;

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(48.000, 34.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 34.500),

                                    new Pose(24.000, 34.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.000, 34.500),

                                    new Pose(56.000, 8.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(48.000, 59.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 59.000),

                                    new Pose(24.000, 59.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.000, 59.000),

                                    new Pose(56.000, 8.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(48.000, 88.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 88.000),

                                    new Pose(24.000, 88.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.000, 88.000),

                                    new Pose(58.000, 100.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 100.000),

                                    new Pose(24.000, 88.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();







        }
    }

    private Paths paths;

    public enum PathState {
        PATH_1,
        WAIT_AFTER_PATH_1,
        PATH_2,
        PATH_3,
        PATH_4,
        WAIT_AFTER_PATH_4,
        PATH_5,
        PATH_6,
        WAIT_AFTER_PATH_6,
        PATH_7,
        PATH_8,
        WAIT_AFTER_PATH_8,
        PATH_9,
        PATH_10,
        WAIT_AFTER_PATH_10,
        PATH_11,
        PATH_12,
        WAIT_AFTER_PATH_12,
        DONE
    }

    private PathState pathState = PathState.PATH_1;

    private void setPathState(PathState newState) {
        pathState = newState;
        if (pathTimer != null) pathTimer.reset();
    }

    private void setShooterVelocityBoth(double totalVel) {
        double perMotor = totalVel / 2.0;
        if (shooter1 != null) shooter1.setVelocity(perMotor);
        if (shooter2 != null) shooter2.setVelocity(perMotor);
    }

    private void updateShooterSimple() {
        boolean enable =
                (pathState == PathState.PATH_1) ||
                        (pathState == PathState.PATH_3) ||
                        (pathState == PathState.PATH_6) ||
                        (pathState == PathState.PATH_9);

        if (enable) {
            setShooterVelocityBoth(SHOOTER_TOTAL_BASE);
        } else {
            setShooterVelocityBoth(0);
        }
    }

    // NO PULSES: shooter ramps, intake just turns on then off
    private void handleEndWait(double t, PathState nextPathState, PathChain nextPath, double idleTime) {
        double rampDuration = 0.9;
        double rampStart = idleTime;
        double totalWaitLimit = 1.7;

        // Shooter ramp 1600 → 1800 total
        if (t < rampStart) {
            setShooterVelocityBoth(SHOOTER_TOTAL_BASE);
        } else {
            double rampT = t - rampStart;
            if (rampT >= rampDuration) {
                setShooterVelocityBoth(SHOOTER_TOTAL_TARGET);
            } else {
                double alpha = rampT / rampDuration;
                double vel = SHOOTER_TOTAL_BASE +
                        (SHOOTER_TOTAL_TARGET - SHOOTER_TOTAL_BASE) * alpha;
                setShooterVelocityBoth(vel);
            }
        }

        // Intake: off during idle, then ON solid until the 2s cap
        if (t < idleTime) {
            intake.setPower(0);
        } else if (t < totalWaitLimit) {
            intake.setPower(INTAKE_POWER);
        } else {
            intake.setPower(0);
            setShooterVelocityBoth(SHOOTER_TOTAL_TARGET);

            if (nextPath != null) {
                follower.followPath(nextPath, true);
                follower.setMaxPower(1.0);
                setPathState(nextPathState);
            } else {
                setPathState(PathState.DONE);
            }
        }
    }

    private void updateStateMachine() {
        switch (pathState) {

            case PATH_1:
                updateShooterSimple();
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_1);
                }
                break;

            case WAIT_AFTER_PATH_1: {
                double t = pathTimer.seconds();
                handleEndWait(t, PathState.PATH_2, paths.Path2, 1.0);
                break;
            }

            case PATH_2:
                updateShooterSimple();
                intake.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_3);
                }
                break;

            // Intake after 1s while driving 3,5,7,9,11
            case PATH_3: {
                updateShooterSimple();
                double t = pathTimer.seconds();
                intake.setPower(t > 1.0 ? INTAKE_POWER : 0);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(paths.Path4, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_4);
                }
                break;
            }

            case PATH_4:
                updateShooterSimple();
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_4);
                }
                break;

            case WAIT_AFTER_PATH_4: {
                double t = pathTimer.seconds();
                handleEndWait(t, PathState.PATH_5, paths.Path5, 1.0);
                break;
            }

            case PATH_5: {
                updateShooterSimple();
                double t = pathTimer.seconds();
                intake.setPower(t > 1.0 ? INTAKE_POWER : 0);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(paths.Path6, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_6);
                }
                break;
            }

            case PATH_6:
                updateShooterSimple();
                intake.setPower(0);
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_6);
                }
                break;

            case WAIT_AFTER_PATH_6: {
                double t = pathTimer.seconds();
                handleEndWait(t, PathState.PATH_7, paths.Path7, 1.0);
                break;
            }

            case PATH_7: {
                updateShooterSimple();
                double t = pathTimer.seconds();
                intake.setPower(t > 1.0 ? INTAKE_POWER : 0);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(paths.Path8, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_8);
                }
                break;
            }

            case PATH_8:
                updateShooterSimple();
                intake.setPower(0);
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_8);
                }
                break;

            case WAIT_AFTER_PATH_8: {
                double t = pathTimer.seconds();
                handleEndWait(t, PathState.PATH_9, paths.Path9, 1.0);
                break;
            }

            case PATH_9: {
                updateShooterSimple();
                double t = pathTimer.seconds();
                intake.setPower(t > 1.0 ? INTAKE_POWER : 0);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(paths.Path10, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_10);
                }
                break;
            }

            case PATH_10:
                updateShooterSimple();
                intake.setPower(0);
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_10);
                }
                break;

            case WAIT_AFTER_PATH_10: {
                double t = pathTimer.seconds();
                handleEndWait(t, PathState.PATH_11, paths.Path11, 1.0);
                break;
            }

            case PATH_11: {
                updateShooterSimple();
                double t = pathTimer.seconds();
                intake.setPower(t > 1.0 ? INTAKE_POWER : 0);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(paths.Path12, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_12);
                }
                break;
            }

            case PATH_12:
                updateShooterSimple();
                intake.setPower(0);
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_12);
                }
                break;

            case WAIT_AFTER_PATH_12: {
                double t = pathTimer.seconds();
                handleEndWait(t, PathState.DONE, null, 1.0);
                break;
            }

            case DONE:
                updateShooterSimple();
                intake.setPower(0);
                break;
        }
    }

    private void updateTurretAim() {
        if (aimMotor == null) return;

        Pose p = follower.getPose();

        double dx = GOAL_X - p.getX();
        double dy = GOAL_Y - p.getY();

        double goalFieldHeading = Math.atan2(dy, dx) + HEADING_OFFSET;
        double desiredTurretRad = AngleUnit.normalizeRadians(goalFieldHeading - p.getHeading());

        int targetTicks = (int) Math.round(desiredTurretRad * AIM_TICKS_PER_RAD);
        targetTicks = clampInt(targetTicks, AIM_MIN_TICKS, AIM_MAX_TICKS);

        aimMotor.setTargetPosition(targetTicks);
    }

    private static int clampInt(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    @Override
    public void init() {
        try {
            pathTimer = new ElapsedTime();
            opModeTimer = new ElapsedTime();

            follower = Constants.createFollower(hardwareMap);
            follower.setPose(new Pose(56, 8, Math.toRadians(180)));

            shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
            shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

            shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
            shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

            shooterServo = hardwareMap.get(Servo.class, "shooterServo");
            shooterServo.setPosition(SHOOTER_SERVO_START_POS);

            intake  = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);

            aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
            aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            aimMotor.setTargetPosition(0);
            aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            aimMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            aimMotor.setPower(AIM_POWER);

            paths = new Paths(follower);

            telemetry.addLine("Blue Auto READY");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("Init Error: " + e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void start() {
        opModeTimer.reset();
        pathTimer.reset();

        setShooterVelocityBoth(0);
        intake.setPower(0);

        if (shooterServo != null) {
            shooterServo.setPosition(SHOOTER_SERVO_START_POS);
        }

        follower.followPath(paths.Path1, true);
        follower.setMaxPower(1.0);
        setPathState(PathState.PATH_1);
    }

    @Override
    public void loop() {
        try {
            follower.update();

            updateStateMachine();
            updateTurretAim();

            telemetry.addData("State", pathState);
            telemetry.addData("Shooter1 Vel", shooter1.getVelocity());
            telemetry.addData("Shooter2 Vel", shooter2.getVelocity());
            telemetry.addData("Shooter1 Pos", shooter1.getCurrentPosition());
            telemetry.addData("Shooter2 Pos", shooter2.getCurrentPosition());
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("Loop Error: " + e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        setShooterVelocityBoth(0);
        if (shooter1 != null) shooter1.setPower(0);
        if (shooter2 != null) shooter2.setPower(0);
        if (intake != null) intake.setPower(0);
        if (aimMotor != null) aimMotor.setPower(0);
    }
}