package org.firstinspires.ftc.teamcode.code;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled

@Autonomous(name = "Red", group = "Autonomous")
@Configurable
public class redauto extends OpMode {

    // ─────────────────────────────────────────────────────────────
    // TUNING
    // ─────────────────────────────────────────────────────────────
    public static int AIM_TICKS = 322;   // RED aims positive

    public static double SHOOTER_TPS = 2500;

    public static double INTAKE_FEED_POWER = -1.0;
    public static double INTAKE_OFF_POWER  = 0.0;

    public static double FEED_START_DELAY = 3.0;
    public static double FEED_ON_TIME     = 0.15;
    public static double FEED_OFF_TIME    = 1.0;
    public static double SORTER_SETTLE    = 0.12;

    public static double PATH2_INTAKE_POWER = -1.0;
    public static double PATH2_INTAKE2_POWER = -1.0;
    public static double PATH2_INTAKE2_BURST_TIME = 0.25;

    // ─────────────────────────────────────────────────────────────
    // Hardware
    // ─────────────────────────────────────────────────────────────
    private Servo sorterServo;
    private Servo shooterServo;
    private RevColorSensorV3 colorSensor;

    private DcMotorEx shooter1;
    private DcMotor aimMotor;

    private DcMotor intakeMotor;    // "intake"  -> FORWARD
    private DcMotor intakeMotor2;   // "intake2" -> REVERSE

    private Limelight3A limelight;

    private final double[] INTAKE_POS = {0.00, 0.32, 0.49};
    private final double[] LAUNCH_POS = {0.22, 0.40, 0.58};

    private static final double SERVO_MIN = 0.3;

    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;
    private Paths paths;

    public enum PathState {
        SHOOT_3_START,   // start: aim + shoot 3
        PATH_1,          // drive 1
        PATH_2,          // drive 2 with intake logic
        PATH_3,          // drive 3
        SHOOT_3_AFTER_P3,// second 3-ball shoot
        DONE             // stop
    }
    private PathState pathState = PathState.SHOOT_3_START;

    private boolean initAimComplete = false;
    private boolean aimHoldEnabled = false;
    private int aimTargetTicks = 0;
    private static final double AIM_OFFSET_POWER = 0.5;

    int intakeIndex = 2;

    private enum ShootRoutineState { SPINUP_WAIT, SORTER_SETTLE, FEED_ON, FEED_OFF, DONE }
    private ShootRoutineState shootState = ShootRoutineState.SPINUP_WAIT;
    private ElapsedTime shootTimer = new ElapsedTime();
    private int shotsFired = 0;

    private boolean path2Entered = false;
    private ElapsedTime path2Timer = new ElapsedTime();

    private boolean path3Started = false;

    private boolean afterFirstShootAimResetDone = false;
    private boolean afterSecondShootAimResetDone = false;

    // ─────────────────────────────────────────────────────────────
    // RED Paths (only 3)
    // ─────────────────────────────────────────────────────────────
    public static class Paths {
        public PathChain Path1, Path2, Path3;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(59.000, 129.000),
                            new Pose(50.000, 113.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(50.000, 113.000),
                            new Pose(28.000, 113.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(28.000, 113.000),
                            new Pose(49.000, 110.000),
                            new Pose(58.000, 118.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-110))
                    .build();
        }
    }

    // ─────────────────────────────────────────────────────────────
    // Helpers
    // ─────────────────────────────────────────────────────────────
    private void setIntake(double power) {
        if (intakeMotor != null) intakeMotor.setPower(power);
    }

    private void setIntake2(double power) {
        if (intakeMotor2 != null) intakeMotor2.setPower(power);
    }

    private void setBothIntakes(double power) {
        setIntake(power);
        setIntake2(power);
    }

    private void setShooterTPS(double tps) {
        shooter1.setVelocity(tps);
    }

    private void stopShooter() {
        shooter1.setVelocity(0);
    }

    public void setPathState(PathState newState) {
        if (newState != pathState) {
            if (newState != PathState.PATH_2) path2Entered = false;
            if (newState != PathState.PATH_3) path3Started = false;
        }
        pathState = newState;
        if (pathTimer != null) pathTimer.reset();
    }

    public void aim(int targetTicks) {
        aimTargetTicks = targetTicks;
        aimHoldEnabled = true;
        aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        aimMotor.setTargetPosition(aimTargetTicks);
        aimMotor.setPower(AIM_OFFSET_POWER);
    }

    private void startShootRoutine() {
        shotsFired = 0;
        shootState = ShootRoutineState.SPINUP_WAIT;
        shootTimer.reset();
    }

    private boolean updateShootRoutine3Balls() {
        setShooterTPS(SHOOTER_TPS);
        sorterServo.setPosition(LAUNCH_POS[intakeIndex]);

        switch (shootState) {
            case SPINUP_WAIT:
                setBothIntakes(INTAKE_OFF_POWER);
                if (shootTimer.seconds() >= FEED_START_DELAY) {
                    shootTimer.reset();
                    shootState = ShootRoutineState.SORTER_SETTLE;
                }
                break;

            case SORTER_SETTLE:
                setBothIntakes(INTAKE_OFF_POWER);
                if (shootTimer.seconds() >= SORTER_SETTLE) {
                    shootTimer.reset();
                    shootState = ShootRoutineState.FEED_ON;
                }
                break;

            case FEED_ON:
                setBothIntakes(INTAKE_FEED_POWER);
                if (shootTimer.seconds() >= FEED_ON_TIME) {
                    shootTimer.reset();
                    shootState = ShootRoutineState.FEED_OFF;
                }
                break;

            case FEED_OFF:
                setBothIntakes(INTAKE_OFF_POWER);
                if (shootTimer.seconds() >= FEED_OFF_TIME) {
                    shotsFired++;
                    intakeIndex = (intakeIndex + 1) % LAUNCH_POS.length;

                    if (shotsFired >= 3) {
                        shootState = ShootRoutineState.DONE;
                    } else {
                        shootTimer.reset();
                        shootState = ShootRoutineState.SORTER_SETTLE;
                    }
                }
                break;

            case DONE:
                setBothIntakes(INTAKE_OFF_POWER);
                stopShooter();
                return true;
        }
        return false;
    }

    private void runIntakeLogic_Path2() {
        setIntake(PATH2_INTAKE_POWER);

        if (!path2Entered) {
            path2Timer.reset();
            path2Entered = true;
        }
        if (path2Timer.seconds() <= PATH2_INTAKE2_BURST_TIME) {
            setIntake2(PATH2_INTAKE2_POWER);
        } else {
            setIntake2(0);
        }
    }

    // ─────────────────────────────────────────────────────────────
    // Auto state machine
    // ─────────────────────────────────────────────────────────────
    public void updateStateMachine() {
        switch (pathState) {

            case SHOOT_3_START:
                if (!initAimComplete) {
                    aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    aim(AIM_TICKS);
                    initAimComplete = true;
                    startShootRoutine();
                }
                if (updateShootRoutine3Balls()) {
                    // After first shooting: return aim to 0
                    if (!afterFirstShootAimResetDone) {
                        aim(0);
                        afterFirstShootAimResetDone = true;
                    }
                    setPathState(PathState.PATH_1);
                }
                break;

//            case PATH_1:
//                setBothIntakes(0);
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path1, true);
//                    follower.setMaxPower(1);
//                    setPathState(PathState.PATH_2);
//                }
//                break;
//
//            case PATH_2:
//                runIntakeLogic_Path2();
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path2, true);
//                    follower.setMaxPower(0.4);
//                    setPathState(PathState.PATH_3);
//                }
//                break;
//
//            case PATH_3:
//                setBothIntakes(0);
//                if (!path3Started) {
//                    follower.followPath(paths.Path3, true);
//                    follower.setMaxPower(1);
//                    path3Started = true;
//                }
//                if (!follower.isBusy()) {
//                    // at end of PATH_3, shoot 3 again
//                    startShootRoutine();
//                    setPathState(PathState.SHOOT_3_AFTER_P3);
//                }
//                break;
//
//            case SHOOT_3_AFTER_P3:
//                if (updateShootRoutine3Balls()) {
//                    // Before auto stops, bring aim back to 0 again
//                    if (!afterSecondShootAimResetDone) {
//                        aim(0);
//                        afterSecondShootAimResetDone = true;
//                    }
//                    setPathState(PathState.DONE);
//                }
//                break;
//
//            case DONE:
//                setBothIntakes(0);
//                stopShooter();
//                if (!follower.isBusy()) telemetry.addLine("RED AUTO COMPLETE ✔");
//                break;
        }
    }

    // ─────────────────────────────────────────────────────────────
    // OpMode lifecycle
    // ─────────────────────────────────────────────────────────────
    @Override
    public void init() {
        try {
            shooterServo = hardwareMap.get(Servo.class, "shooterServo");
            shooterServo.setPosition(SERVO_MIN);

            sorterServo = hardwareMap.get(Servo.class, "sortservo");
            sorterServo.setPosition(INTAKE_POS[0]);

            colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");

            shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
            shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

            PIDFCoefficients pidf = new PIDFCoefficients(5, 0, 0, 19);
            shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2");

            // intake forward, intake2 reverse, negative power used
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

            setBothIntakes(0);

            aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
            aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            aimMotor.setDirection(DcMotor.Direction.REVERSE);
            aimMotor.setPower(0);

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(3);
            limelight.start();

            pathTimer = new ElapsedTime();
            opModeTimer = new ElapsedTime();

            follower = Constants.createFollower(hardwareMap);
            follower.setPose(new Pose(59.0, 129.0, Math.toRadians(180)));
            paths = new Paths(follower);

            telemetry.addLine("✅ RED AUTO (3 paths, aim reset at start/after P3)");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("❌ Init Error: " + e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void start() {
        opModeTimer.reset();
        setPathState(PathState.SHOOT_3_START);

        intakeIndex = 2;
        initAimComplete = false;

        startShootRoutine();

        path2Entered = false;
        path3Started = false;
        afterFirstShootAimResetDone = false;
        afterSecondShootAimResetDone = false;

        setBothIntakes(0);
    }

    @Override
    public void loop() {
        try {
            follower.update();
            updateStateMachine();

            if (aimHoldEnabled) {
                if (aimMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    aimMotor.setTargetPosition(aimTargetTicks);
                    aimMotor.setPower(AIM_OFFSET_POWER);
                }
            }

            telemetry.addData("Alliance", "RED");
            telemetry.addData("State", pathState);
            telemetry.addData("AimPos", aimMotor.getCurrentPosition());
            telemetry.addData("Shooter target tps", (int) SHOOTER_TPS);
            telemetry.addData("Shooter actual tps", (int) shooter1.getVelocity());
            telemetry.addData("ShotsFired", shotsFired);
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("Loop Error: " + e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        if (aimMotor != null) aimMotor.setPower(0);
        if (shooter1 != null) stopShooter();
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (intakeMotor2 != null) intakeMotor2.setPower(0);
    }
}