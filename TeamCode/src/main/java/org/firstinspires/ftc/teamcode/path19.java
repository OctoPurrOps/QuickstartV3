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

@Autonomous(name = "BLUENewPath19ball", group = "Autonomous")
public class path19 extends OpMode {

    // Pedro follower + timers
    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;

    // Shooter
    private DcMotorEx shooter;
    private static final double SHOOTER_VELOCITY = 1600.0;
    private static final double SHOOTER_TARGET_AFTER_PATH1 = 1800.0;

    // Shooter servo
    private Servo shooterServo;
    private static final double SHOOTER_SERVO_START_POS = 0.03;

    // Intakes
    private DcMotorEx intake;
    private DcMotorEx intake2;
    private static final double INTAKE_POWER = -1.0;

    // Turret / aim motor
    private DcMotor aimMotor;

    // Turret aiming constants
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;
    private static final double HEADING_OFFSET = Math.toRadians(0);
    private static final double TICKS_PER_REV = 1627;
    private static final double AIM_TICKS_PER_RAD = TICKS_PER_REV / (2.0 * Math.PI);
    private static final int AIM_MIN_TICKS = -2000;
    private static final int AIM_MAX_TICKS = 2000;
    private static final double AIM_POWER = 1;

    // New paths holder
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
                                    new Pose(27.000, 130.000),
                                    new Pose(58.000, 100.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(144))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 100.000),
                                    new Pose(50.000, 70.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.000, 70.000),
                                    new Pose(24.000, 70.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.000, 70.000),
                                    new Pose(58.000, 100.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 100.000),
                                    new Pose(12.000, 70.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.000, 70.000),
                                    new Pose(58.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 100.000),
                                    new Pose(12.000, 70.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.000, 70.000),
                                    new Pose(58.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.000, 100.000),
                                    new Pose(47.000, 84.000),
                                    new Pose(26.000, 84.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(26.000, 84.000),
                                    new Pose(58.000, 100.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 100.000),
                                    new Pose(12.000, 70.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.000, 70.000),
                                    new Pose(58.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();
        }
    }

    private Paths paths;

    // State machine: each PATH_i has a matching WAIT_AFTER_PATH_i
    public enum PathState {
        PATH_1, WAIT_AFTER_PATH_1,
        PATH_2, WAIT_AFTER_PATH_2,
        PATH_3, WAIT_AFTER_PATH_3,
        PATH_4, WAIT_AFTER_PATH_4,
        PATH_5, WAIT_AFTER_PATH_5,
        PATH_6, WAIT_AFTER_PATH_6,
        PATH_7, WAIT_AFTER_PATH_7,
        PATH_8, WAIT_AFTER_PATH_8,
        PATH_9, WAIT_AFTER_PATH_9,
        PATH_10, WAIT_AFTER_PATH_10,
        PATH_11, WAIT_AFTER_PATH_11,
        PATH_12, WAIT_AFTER_PATH_12,
        DONE
    }

    private PathState pathState = PathState.PATH_1;

    private void setPathState(PathState newState) {
        pathState = newState;
        if (pathTimer != null) pathTimer.reset();
    }

    // Shooter base helper: only on while driving certain paths if you want
    private void updateShooterSimple() {
        boolean enable =
                (pathState == PathState.PATH_1) ||
                        (pathState == PathState.PATH_2) ||
                        (pathState == PathState.PATH_3) ||
                        (pathState == PathState.PATH_4) ||
                        (pathState == PathState.PATH_5) ||
                        (pathState == PathState.PATH_6) ||
                        (pathState == PathState.PATH_7) ||
                        (pathState == PathState.PATH_8) ||
                        (pathState == PathState.PATH_9) ||
                        (pathState == PathState.PATH_10) ||
                        (pathState == PathState.PATH_11) ||
                        (pathState == PathState.PATH_12);

        if (enable) {
            shooter.setVelocity(SHOOTER_VELOCITY);
        } else {
            shooter.setVelocity(0);
        }
    }

    // Shared wait logic: idle + pulses + shooter ramp + solid intake
    private void handleWaitAfterPath(double t, PathState nextPathState, PathChain nextPath, double idleTime) {
        double pulseLength = 0.1;
        int numPulses = 3;
        double pulseWindow = numPulses * 2 * pulseLength; // 0.6 s

        double rampDuration = 1.3;
        double rampStart = idleTime; // ramp in sync with pulses

        // Shooter
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

        // Intakes
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

            // Driving states: follow path, intake ON, then go to wait state

            case PATH_1:
                updateShooterSimple();
                intake.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    setPathState(PathState.WAIT_AFTER_PATH_1);
                }
                break;

            case WAIT_AFTER_PATH_1: {
                double t = pathTimer.seconds();
                double idleTime = 1.0;
                handleWaitAfterPath(t, PathState.PATH_2, paths.Path2, idleTime);
                break;
            }

            case PATH_2:
                updateShooterSimple();
                intake.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    setPathState(PathState.WAIT_AFTER_PATH_2);
                }
                break;

            case WAIT_AFTER_PATH_2: {
                double t = pathTimer.seconds();
                double idleTime = 1.0;
                handleWaitAfterPath(t, PathState.PATH_3, paths.Path3, idleTime);
                break;
            }

            case PATH_3:
                updateShooterSimple();
                intake.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    setPathState(PathState.WAIT_AFTER_PATH_3);
                }
                break;

            case WAIT_AFTER_PATH_3: {
                double t = pathTimer.seconds();
                double idleTime = 1.0;
                handleWaitAfterPath(t, PathState.PATH_4, paths.Path4, idleTime);
                break;
            }

            case PATH_4:
                updateShooterSimple();
                intake.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    setPathState(PathState.WAIT_AFTER_PATH_4);
                }
                break;

            case WAIT_AFTER_PATH_4: {
                double t = pathTimer.seconds();
                double idleTime = 1.0;
                handleWaitAfterPath(t, PathState.PATH_5, paths.Path5, idleTime);
                break;
            }

            case PATH_5:
                updateShooterSimple();
                intake.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    setPathState(PathState.WAIT_AFTER_PATH_5);
                }
                break;

            case WAIT_AFTER_PATH_5: {
                double t = pathTimer.seconds();
                double idleTime = 1.0;
                handleWaitAfterPath(t, PathState.PATH_6, paths.Path6, idleTime);
                break;
            }

            case PATH_6:
                updateShooterSimple();
                intake.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    setPathState(PathState.WAIT_AFTER_PATH_6);
                }
                break;

            case WAIT_AFTER_PATH_6: {
                double t = pathTimer.seconds();
                double idleTime = 1.0;
                handleWaitAfterPath(t, PathState.PATH_7, paths.Path7, idleTime);
                break;
            }

            case PATH_7:
                updateShooterSimple();
                intake.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    setPathState(PathState.WAIT_AFTER_PATH_7);
                }
                break;

            case WAIT_AFTER_PATH_7: {
                double t = pathTimer.seconds();
                double idleTime = 1.0;
                handleWaitAfterPath(t, PathState.PATH_8, paths.Path8, idleTime);
                break;
            }

            case PATH_8:
                updateShooterSimple();
                intake.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    setPathState(PathState.WAIT_AFTER_PATH_8);
                }
                break;

            case WAIT_AFTER_PATH_8: {
                double t = pathTimer.seconds();
                double idleTime = 1.0;
                handleWaitAfterPath(t, PathState.PATH_9, paths.Path9, idleTime);
                break;
            }

            case PATH_9:
                updateShooterSimple();
                intake.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    setPathState(PathState.WAIT_AFTER_PATH_9);
                }
                break;

            case WAIT_AFTER_PATH_9: {
                double t = pathTimer.seconds();
                double idleTime = 1.0;
                handleWaitAfterPath(t, PathState.PATH_10, paths.Path10, idleTime);
                break;
            }

            case PATH_10:
                updateShooterSimple();
                intake.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    setPathState(PathState.WAIT_AFTER_PATH_10);
                }
                break;

            case WAIT_AFTER_PATH_10: {
                double t = pathTimer.seconds();
                double idleTime = 1.0;
                handleWaitAfterPath(t, PathState.PATH_11, paths.Path11, idleTime);
                break;
            }

            case PATH_11:
                updateShooterSimple();
                intake.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    setPathState(PathState.WAIT_AFTER_PATH_11);
                }
                break;

            case WAIT_AFTER_PATH_11: {
                double t = pathTimer.seconds();
                double idleTime = 1.0;
                handleWaitAfterPath(t, PathState.PATH_12, paths.Path12, idleTime);
                break;
            }

            case PATH_12:
                updateShooterSimple();
                intake.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    setPathState(PathState.WAIT_AFTER_PATH_12);
                }
                break;

            case WAIT_AFTER_PATH_12: {
                double t = pathTimer.seconds();
                double idleTime = 1.0;
                handleWaitAfterPath(t, PathState.DONE, null, idleTime);
                break;
            }

            case DONE:
                shooter.setVelocity(0);
                intake.setPower(0);
                intake2.setPower(0);
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
            follower.setPose(new Pose(27, 130, Math.toRadians(144)));

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