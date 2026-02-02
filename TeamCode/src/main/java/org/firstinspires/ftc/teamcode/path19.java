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

@Autonomous(name = "blue19", group = "Autonomous")
public class path19 extends OpMode {

    // Pedro follower + timers
    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;

    // Shooter
    private DcMotorEx shooter;
    private static final double SHOOTER_VELOCITY = 1600.0;
    private static final double SHOOTER_TARGET_AFTER_PATH = 1800.0;

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

    // Paths holder
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

    // State machine
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

    // Shooter helper: base speed on driving paths you want
    private void updateShooterSimple() {
        boolean enable =
                (pathState == PathState.PATH_1) ||
                        (pathState == PathState.PATH_3) ||
                        (pathState == PathState.PATH_6) ||
                        (pathState == PathState.PATH_9);

        if (enable) {
            shooter.setVelocity(SHOOTER_VELOCITY);
        } else {
            shooter.setVelocity(0);
        }
    }

    // Shared wait logic at ends of 1,4,6,8,10,12:
    // idle + intake pulses + shooter ramp, but total capped at 2s.
    private void handleEndWait(double t, PathState nextPathState, PathChain nextPath, double idleTime) {
        double pulseLength = 0.1;
        int numPulses = 3;
        double pulseWindow = numPulses * 2 * pulseLength; // 0.6 s

        double rampDuration = 1.3;
        double rampStart = idleTime; // ramp while pulses run

        double totalWaitLimit = 2.0; // cap end wait at 2 seconds

        // Shooter ramp
        if (t < rampStart) {
            shooter.setVelocity(SHOOTER_VELOCITY);
        } else {
            double rampT = t - rampStart;
            if (rampT >= rampDuration) {
                shooter.setVelocity(SHOOTER_TARGET_AFTER_PATH);
            } else {
                double alpha = rampT / rampDuration;
                double vel = SHOOTER_VELOCITY +
                        (SHOOTER_TARGET_AFTER_PATH - SHOOTER_VELOCITY) * alpha;
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

        } else if (t < totalWaitLimit) {
            // after pulses until 2s total: solid intake on
            intake.setPower(INTAKE_POWER);
            intake2.setPower(INTAKE_POWER);

        } else {
            // >=2s: stop and advance
            intake.setPower(0);
            intake2.setPower(0);
            shooter.setVelocity(SHOOTER_TARGET_AFTER_PATH);

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

            // PATH 1 (Group A end)
            case PATH_1:
                updateShooterSimple();
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_1);
                }
                break;

            case WAIT_AFTER_PATH_1: {
                double t = pathTimer.seconds();
                double idleTime1 = 1.0; // idle portion of the 2s
                handleEndWait(t, PathState.PATH_2, paths.Path2, idleTime1);
                break;
            }

            // PATH 2 (Group B rule applies to Path3, not Path2)
            case PATH_2:
                updateShooterSimple();
                // no special intake here, just off
                intake.setPower(0);
                intake2.setPower(0);

                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_3);
                }
                break;

            // PATH 3 (Group B: intake starts after 1s while moving)
            case PATH_3: {
                updateShooterSimple();
                double t = pathTimer.seconds();
                if (t > 1.0) {
                    intake.setPower(INTAKE_POWER);
                    intake2.setPower(INTAKE_POWER);
                } else {
                    intake.setPower(0);
                    intake2.setPower(0);
                }

                if (!follower.isBusy()) {
                    // stop intake before next path
                    intake.setPower(0);
                    intake2.setPower(0);
                    follower.followPath(paths.Path4, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_4);
                }
                break;
            }

            // PATH 4 (Group A end)
            case PATH_4:
                updateShooterSimple();
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_4);
                }
                break;

            case WAIT_AFTER_PATH_4: {
                double t = pathTimer.seconds();
                double idleTime4 = 1.0;
                handleEndWait(t, PathState.PATH_5, paths.Path5, idleTime4);
                break;
            }

            // PATH 5 (Group B)
            case PATH_5: {
                updateShooterSimple();
                double t = pathTimer.seconds();
                if (t > 1.0) {
                    intake.setPower(INTAKE_POWER);
                    intake2.setPower(INTAKE_POWER);
                } else {
                    intake.setPower(0);
                    intake2.setPower(0);
                }

                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    follower.followPath(paths.Path6, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_6);
                }
                break;
            }

            // PATH 6 (Group A end)
            case PATH_6:
                updateShooterSimple();
                intake.setPower(0);
                intake2.setPower(0);

                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_6);
                }
                break;

            case WAIT_AFTER_PATH_6: {
                double t = pathTimer.seconds();
                double idleTime6 = 1.0;
                handleEndWait(t, PathState.PATH_7, paths.Path7, idleTime6);
                break;
            }

            // PATH 7 (Group B)
            case PATH_7: {
                updateShooterSimple();
                double t = pathTimer.seconds();
                if (t > 1.0) {
                    intake.setPower(INTAKE_POWER);
                    intake2.setPower(INTAKE_POWER);
                } else {
                    intake.setPower(0);
                    intake2.setPower(0);
                }

                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    follower.followPath(paths.Path8, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_8);
                }
                break;
            }

            // PATH 8 (Group A end)
            case PATH_8:
                updateShooterSimple();
                intake.setPower(0);
                intake2.setPower(0);

                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_8);
                }
                break;

            case WAIT_AFTER_PATH_8: {
                double t = pathTimer.seconds();
                double idleTime8 = 1.0;
                handleEndWait(t, PathState.PATH_9, paths.Path9, idleTime8);
                break;
            }

            // PATH 9 (Group B)
            case PATH_9: {
                updateShooterSimple();
                double t = pathTimer.seconds();
                if (t > 1.0) {
                    intake.setPower(INTAKE_POWER);
                    intake2.setPower(INTAKE_POWER);
                } else {
                    intake.setPower(0);
                    intake2.setPower(0);
                }

                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    follower.followPath(paths.Path10, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_10);
                }
                break;
            }

            // PATH 10 (Group A end)
            case PATH_10:
                updateShooterSimple();
                intake.setPower(0);
                intake2.setPower(0);

                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_10);
                }
                break;

            case WAIT_AFTER_PATH_10: {
                double t = pathTimer.seconds();
                double idleTime10 = 1.0;
                handleEndWait(t, PathState.PATH_11, paths.Path11, idleTime10);
                break;
            }

            // PATH 11 (Group B)
            case PATH_11: {
                updateShooterSimple();
                double t = pathTimer.seconds();
                if (t > 1.0) {
                    intake.setPower(INTAKE_POWER);
                    intake2.setPower(INTAKE_POWER);
                } else {
                    intake.setPower(0);
                    intake2.setPower(0);
                }

                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    follower.followPath(paths.Path12, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_12);
                }
                break;
            }

            // PATH 12 (Group A end, final)
            case PATH_12:
                updateShooterSimple();
                intake.setPower(0);
                intake2.setPower(0);

                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AFTER_PATH_12);
                }
                break;

            case WAIT_AFTER_PATH_12: {
                double t = pathTimer.seconds();
                double idleTime12 = 1.0;
                // no next path, so null and DONE
                handleEndWait(t, PathState.DONE, null, idleTime12);
                break;
            }

            case DONE:
                updateShooterSimple();
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
