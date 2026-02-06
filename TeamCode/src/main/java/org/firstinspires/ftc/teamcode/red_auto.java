package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
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

@Autonomous(name = "RED_auto", group = "Autonomous")
public class red_auto extends OpMode {

    // Pedro follower + timers
    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;

    // Shooter
    private DcMotorEx shooter;
    private static final double SHOOTER_VELOCITY = 1600.0;
    private static final double SHOOTER_TARGET_AFTER_PATH1 = 1700.0;

    // Shooter servo
    private Servo shooterServo;
    private static final double SHOOTER_SERVO_START_POS = 0.1;

    // Intakes
    private DcMotorEx intake;
    private DcMotorEx intake2;
    private static final double INTAKE_POWER = -1.0;

    // Turret / aim motor
    private DcMotor aimMotor;

    // Turret aiming constants
    private static final double GOAL_X = 144.0;
    private static final double GOAL_Y = 144.0;
    private static final double HEADING_OFFSET = Math.toRadians(0);
    private static final double TICKS_PER_REV = 1627;
    private static final double AIM_TICKS_PER_RAD = TICKS_PER_REV / (2.0 * Math.PI);
    private static final int AIM_MIN_TICKS = -2000;
    private static final int AIM_MAX_TICKS = 2000;
    private static final double AIM_POWER = 1;

    // Paths holder
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(117, 130.000),

                                    new Pose(86, 100.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(36))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86, 100.000),

                                    new Pose(114, 85.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(114, 85.000),

                                    new Pose(86, 100.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86, 100.000),

                                    new Pose(95, 67.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95, 67.000),

                                    new Pose(114, 67.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(114, 67.000),

                                    new Pose(86, 100.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86, 100.000),

                                    new Pose(97, 44.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(97, 44.000),

                                    new Pose(114, 44.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(114, 44.000),

                                    new Pose(86, 100.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();







            // Optional Path10/11 definitions here if you use them
        }
    }

    private Paths paths;

    // State machine
    public enum PathState {
        PATH_1,
        WAIT_AFTER_PATH_1,
        PATH_2,
        PATH_3,
        WAIT_AFTER_PATH_3,
        PATH_4,
        PATH_5,
        PATH_6,
        WAIT_AFTER_PATH_6,
        PATH_7,
        PATH_8,
        END_AT_PATH_8,
        PATH_9,
        WAIT_AFTER_PATH_9,   // NEW
        PATH_10,
        PATH_11,
        DONE
    }

    private PathState pathState = PathState.PATH_1;

    private void setPathState(PathState newState) {
        pathState = newState;
        if (pathTimer != null) pathTimer.reset();
    }

    // Shooter helper
    private void updateShooterSimple() {
        boolean enable =
                (pathState == PathState.PATH_1) ||
                        (pathState == PathState.PATH_3) ||
                        (pathState == PathState.PATH_6) ||
                        (pathState == PathState.PATH_9) ||
                        (pathState == PathState.WAIT_AFTER_PATH_6) ||
                        (pathState == PathState.END_AT_PATH_8);

        if (enable) {
            shooter.setVelocity(SHOOTER_VELOCITY);
        } else {
            shooter.setVelocity(0);
        }
    }

    // Shared wait logic: used for Path1, Path3, Path6, Path9
    private void handleWaitAfterPath(double t, PathState nextPathState, PathChain nextPath, double idleTime) {
        double pulseLength = 0.1;
        int numPulses = 3;
        double pulseWindow = numPulses * 2 * pulseLength; // 0.6 s

        double rampDuration = 1.3;
        double rampStart = idleTime; // ramp with pulses

        // Shooter ramp
        if (t < rampStart) {
            shooter.setVelocity(SHOOTER_VELOCITY);
        } else {
            double rampT = t - rampStart;
            if (rampT >= rampDuration) {
                shooter.setVelocity(SHOOTER_TARGET_AFTER_PATH1);
            } else {
                double alpha = rampT / rampDuration;
                double vel = SHOOTER_VELOCITY +
                        (SHOOTER_TARGET_AFTER_PATH1 - SHOOTER_VELOCITY) * alpha;
                shooter.setVelocity(vel);
            }
        }

        // Intake behavior
        if (t < idleTime) {
            intake.setPower(0);
            intake2.setPower(0);

        } else if (t < idleTime + pulseWindow) {
            double localT = t - idleTime;
            double phase = localT % (2 * pulseLength);
            boolean on = phase < pulseLength;

            if (on) {
                intake.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);
            } else {
                intake.setPower(0);
                intake2.setPower(0);
            }

        } else if (t < 5.0) {
            intake.setPower(INTAKE_POWER);
            intake2.setPower(INTAKE_POWER);

        } else {
            intake.setPower(0);
            intake2.setPower(0);
            shooter.setVelocity(SHOOTER_TARGET_AFTER_PATH1);

            if (nextPath != null) {
                follower.followPath(nextPath, true);
                follower.setMaxPower(1.0);
                setPathState(nextPathState);
            } else {
                setPathState(PathState.DONE);
            }
        }
    }

    // -------------------- STATE MACHINE --------------------
    private void updateStateMachine() {
        switch (pathState) {

            // PATH 1
            case PATH_1:
                updateShooterSimple();
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_1);
                }
                break;

            case WAIT_AFTER_PATH_1: {
                double t1 = pathTimer.seconds();
                double idleTime1 = 0.2;
                handleWaitAfterPath(t1, PathState.PATH_2, paths.Path2, idleTime1);
                break;
            }

            // PATH 2
            case PATH_2:
                updateShooterSimple();
                if (pathTimer.seconds() < 0.1) {
                    intake2.setPower(INTAKE_POWER);
                    intake.setPower(INTAKE_POWER);
                } else {
                    intake2.setPower(0);
                    intake.setPower(INTAKE_POWER);
                }

                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(paths.Path3, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_3);
                }
                break;

            // PATH 3
            case PATH_3:
                updateShooterSimple();
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_3);
                }
                break;

            case WAIT_AFTER_PATH_3: {
                double t3 = pathTimer.seconds();
                double idleTime3 = 1.0;
                handleWaitAfterPath(t3, PathState.PATH_4, paths.Path4, idleTime3);
                break;
            }

            // PATH 4
            case PATH_4:
                updateShooterSimple();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_5);
                }
                break;

            // PATH 5
            case PATH_5:
                updateShooterSimple();
                if (pathTimer.seconds() < 0.1) {
                    intake2.setPower(INTAKE_POWER);
                    intake.setPower(INTAKE_POWER);
                } else {
                    intake2.setPower(0);
                    intake.setPower(INTAKE_POWER);
                }

                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(paths.Path6, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_6);
                }
                break;

            // PATH 6
            case PATH_6:
                updateShooterSimple();
                intake.setPower(0);
                intake2.setPower(0);

                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_6);
                }
                break;

            case WAIT_AFTER_PATH_6: {
                double t6 = pathTimer.seconds();
                double idleTime6 = 1.0;
                handleWaitAfterPath(t6, PathState.PATH_7, paths.Path7, idleTime6);
                break;
            }

            // PATH 7
            case PATH_7:
                updateShooterSimple();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_8);
                }
                break;

            // PATH 8
            case PATH_8:
                updateShooterSimple();
                if (pathTimer.seconds() < 0.1) {
                    intake2.setPower(INTAKE_POWER);
                    intake.setPower(INTAKE_POWER);
                } else {
                    intake2.setPower(0);
                    intake.setPower(INTAKE_POWER);
                }

                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_9);
                }
                break;

            // PATH 9
            case PATH_9:
                updateShooterSimple();
                intake.setPower(0);
                intake2.setPower(0);

                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_9);
                }
                break;

            case WAIT_AFTER_PATH_9: {
                // same logic as Path6 wait, but its own idle time and next path (Path10)
                double t9 = pathTimer.seconds();
                double idleTime9 = 1.0; // adjust as you like
                PathChain next = (paths.Path10 != null) ? paths.Path10 : null;
                PathState nextState = (paths.Path10 != null) ? PathState.PATH_10 : PathState.DONE;
                handleWaitAfterPath(t9, nextState, next, idleTime9);
                break;
            }

            // PATH 10 / 11 (if used)
            case PATH_10:
                updateShooterSimple();
                if (!follower.isBusy()) {
                    if (paths.Path11 != null) {
                        follower.followPath(paths.Path11, true);
                        follower.setMaxPower(1.0);
                        setPathState(PathState.PATH_11);
                    } else {
                        setPathState(PathState.DONE);
                    }
                }
                break;

            case PATH_11:
                updateShooterSimple();
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case END_AT_PATH_8:
                updateShooterSimple();
                intake.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);
                break;

            case DONE:
                updateShooterSimple();
                break;
        }
    }

    // -------------------- TURRET AIM LOGIC --------------------
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

    // -------------------- LIFECYCLE --------------------
    @Override
    public void init() {
        try {
            pathTimer = new ElapsedTime();
            opModeTimer = new ElapsedTime();

            follower = Constants.createFollower(hardwareMap);
            follower.setPose(new Pose(117, 130, Math.toRadians(36)));

            shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter.setDirection(DcMotorSimple.Direction.REVERSE);

            shooterServo = hardwareMap.get(Servo.class, "shooterServo");
            shooterServo.setPosition(SHOOTER_SERVO_START_POS);

            intake  = hardwareMap.get(DcMotorEx.class, "intake");
            intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake2.setDirection(DcMotorSimple.Direction.REVERSE);

            aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
            aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            aimMotor.setTargetPosition(0);
            aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            aimMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            aimMotor.setPower(AIM_POWER);

            paths = new Paths(follower);

            telemetry.addLine("✅ Blue Auto READY");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("❌ Init Error: " + e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void start() {
        opModeTimer.reset();
        pathTimer.reset();

        shooter.setVelocity(0);

        intake.setPower(0);
        intake2.setPower(0);

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
            telemetry.addData("Pose", follower.getPose());
            telemetry.addData("OpMode Time", opModeTimer.seconds());
            telemetry.addData("State Time", pathTimer.seconds());
            telemetry.addData("Shooter Vel Start", SHOOTER_VELOCITY);
            telemetry.addData("Shooter Vel Actual", shooter.getVelocity());
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Intake2 Power", intake2.getPower());
            telemetry.addData("Servo Pos", shooterServo != null ? shooterServo.getPosition() : -1);
            telemetry.addData("Aim Target", aimMotor.getTargetPosition());
            telemetry.addData("Aim Current", aimMotor.getCurrentPosition());
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("Loop Error: " + e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        if (shooter != null) {
            shooter.setVelocity(0);
            shooter.setPower(0);
        }
        if (intake != null) intake.setPower(0);
        if (intake2 != null) intake2.setPower(0);
        if (aimMotor != null) aimMotor.setPower(0);
    }
}