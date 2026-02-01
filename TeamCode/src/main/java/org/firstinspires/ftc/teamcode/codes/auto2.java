package org.firstinspires.ftc.teamcode.codes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
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
@Autonomous(name = "🔵 BLUE-AUTO (TeleOp Shooter+LL)", group = "Autonomous")
@Configurable
public class auto2 extends OpMode {

    // ===== Hardware =====
    private Servo pushServo;
    private Servo sorterServo;
    private RevColorSensorV3 colorSensor;
    private DcMotorEx shooter1;
    private DcMotor aimMotor;
    private DcMotor intakeMotor;
    private Servo shooterServo;
    private Limelight3A limelight;

    // ===== ADDED: your tuned values =====
    private static final int PATH1_AIM_TICKS = -322;
    private static final double PATH1_SHOOTER_TARGET_TPS = 6000.0;

    // Path1: servo max then shooter start sequencing
    private ElapsedTime path1SeqTimer = new ElapsedTime();
    private boolean path1ServoWasSet = false;
    private boolean path1ShooterStarted = false;
    private static final double PATH1_SERVO_SETTLE_SECONDS = 0.12;

    // ===== Sorter positions (keep yours) =====
    private final double[] INTAKE_POS = {0.22, 0.40, 0.58} ;
    private final double[] LAUNCH_POS = {0.00, 0.32, 0.49};

    // ===== Push servo =====
    static final double PUSH_UP_POS = 1.0;
    static final double PUSH_DOWN_POS = 0.0;
    private static final double PUSH_HOME = 0.0;

    // ===== State / counters =====
    static final int MAX_BALLS = 3;

    int ballCount = 0;
    int intakeIndex = 0;

    boolean intakeFull = false;
    boolean ballDetected = false;

    // Color thresholds (keep yours)
    static final int BALL_DETECT = 280;
    static final int BALL_CLEAR = 250;

    // Timing
    ElapsedTime ballTimer = new ElapsedTime();
    ElapsedTime servoMoveTimer = new ElapsedTime();

    ElapsedTime forceIndexTimer = new ElapsedTime();
    boolean useForceIndex = false;
    static final double FORCE_INDEX_INTERVAL = 0.6;

    static final double BALL_COOLDOWN = 0.15;
    static final double REARM_TIMEOUT = 0.20;
    static final double SERVO_SETTLE_TIME = 0.08;

    // ===== Launch control =====
    private boolean launchMode = false;
    private boolean launching = false;

    private boolean servoMoving = false;

    enum LaunchState {
        IDLE, ROTATE, WAIT_SETTLE, PUSH_UP, WAIT_UP, PUSH_DOWN, WAIT_DOWN
    }

    LaunchState launchState = LaunchState.IDLE;
    ElapsedTime launchTimer = new ElapsedTime();

    // Match your Auto times
    private static final double SETTLE_TIME = 0.45;
    static final double PUSH_UP_TIME = 0.25;
    static final double PUSH_DOWN_TIME = 0.35;

    // ===== Turret hold / aim =====
    private static final double AIM_OFFSET_POWER = 0.3;
    private boolean aimHoldEnabled = false;
    private int aimTargetTicks = 0;
    private boolean initAimComplete = false;

    // ===== Limelight: TeleOp logic port =====
    private static final double DEADZONE = 2.0;
    private static final double MAX_TURRET_POWER = 0.35;
    private static final double FILTER_ALPHA_TX = 0.85;
    private static final double KP_TX = 0.025;
    private double filteredTx = 0.0;

    static final long MAX_STALE_MS = 80;
    static final double TA_MIN = 0.15;

    // ===== Distance model + mapping (TeleOp values) =====
    private static final double CAMERA_HEIGHT = 0.25;   // meters
    private static final double TARGET_HEIGHT = 0.90;   // meters
    private static final double CAMERA_ANGLE  = 20.0;   // degrees

    private static final double SERVO_MIN = 0.30;
    private static final double SERVO_MAX = 0.45;
    private static final double DIST_MIN  = 0.40;
    private static final double DIST_MAX  = 3.00;

    private static final double RPM_MIN = 200;
    private static final double RPM_MAX = 1800;

    // Shooter command storage
    private double targetShooterRPM = 900;

    // Motor conversion (ticks/rev from motor type)
    private double shooterTicksPerRev = 28.0;

    // Allow fixed mode like your old auto did
    private boolean useFixedShooterVelocity = false;

    // ===== Pedro Pathing =====
    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;

    private ElapsedTime path3Timer = new ElapsedTime();
    private boolean path3DelayStarted = false;

    private static final double PATH3_DELAY_SECONDS = 4.5;
    private static final double PATH1_DELAY_SECONDS = 8.0;
    public static boolean USE_PATH3 = true;

    public enum PathState {
        PATH_1, PATH_2, PATH_3, PATH_4, PATH_5, PATH_6, PATH_7, PATH_8, PATH_9, DONE
    }

    private PathState pathState = PathState.PATH_1;
    private Paths paths;

    // ===== Paths (UPDATED Path1 with callbacks) =====
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

        public Paths(Follower follower, auto2 opMode) {

            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 16.000),

                                    new Pose(52.000, 37.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(52.000, 37.000),

                                    new Pose(16.000, 37.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.000, 37.000),
                                    new Pose(49.000, 34.000),
                                    new Pose(59.000, 13.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.000, 13.000),

                                    new Pose(47.000, 50.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.000, 50.000),

                                    new Pose(28.000, 50.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(28.000, 50.000),

                                    new Pose(47.000, 67.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.000, 67.000),

                                    new Pose(28.000, 67.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(28.000, 67.000),

                                    new Pose(47.000, 67.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.000, 67.000),

                                    new Pose(47.000, 67.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

        }
    }

    // ===== Helpers =====
    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void resetBallState() {
        ballCount = 0;
        intakeIndex = 0;
        ballDetected = false;
        intakeFull = false;
        launchMode = false;
        launching = false;
        servoMoving = false;
        useForceIndex = false;

        sorterServo.setPosition(INTAKE_POS[0]);

        ballTimer.reset();
        servoMoveTimer.reset();
        forceIndexTimer.reset();
    }

    // setVelocity() is ticks/second. [web:36]
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * shooterTicksPerRev;
    }

    private void setShooterRPM(double rpm) {
        targetShooterRPM = rpm;
        shooter1.setVelocity(rpmToTicksPerSecond(rpm));
    }

    private void stopShooter() {
        shooter1.setVelocity(0);
    }

    private double distanceFromTy(double tyDeg) {
        double angleRad = Math.toRadians(CAMERA_ANGLE + tyDeg);
        return (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRad);
    }

    private double mapDistanceToServo(double distM) {
        double servoPos = SERVO_MIN + (distM - DIST_MIN) / (DIST_MAX - DIST_MIN) * (SERVO_MAX - SERVO_MIN);
        return clamp(servoPos, SERVO_MIN, SERVO_MAX);
    }

    private double mapDistanceToRPM(double distM) {
        double rpm = RPM_MIN + (distM - DIST_MIN) / (DIST_MAX - DIST_MIN) * (RPM_MAX - RPM_MIN);
        return clamp(rpm, RPM_MIN, RPM_MAX);
    }

    private void updateLimelightAimAndShooter(boolean enableTurretAim, boolean enableShooter) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        if (result.getStaleness() > MAX_STALE_MS) return;
        if (result.getTa() < TA_MIN) return;

        double tx = result.getTx();
        double ty = result.getTy();

        if (enableTurretAim) {
            filteredTx = FILTER_ALPHA_TX * filteredTx + (1.0 - FILTER_ALPHA_TX) * tx;

            double turretPower = 0.0;
            if (Math.abs(filteredTx) > DEADZONE) {
                turretPower = filteredTx * KP_TX;
            }
            turretPower = clamp(turretPower, -MAX_TURRET_POWER, MAX_TURRET_POWER);
            aimMotor.setPower(turretPower);
        }

        double dist = distanceFromTy(ty);
        shooterServo.setPosition(mapDistanceToServo(dist));

        if (enableShooter) {
            setShooterRPM(mapDistanceToRPM(dist));
        }
    }

    // ===== Ball/intake sensor =====
    public void updateBallSensor(int brightness) {
        boolean canStep = !servoMoving && (servoMoveTimer.seconds() > SERVO_SETTLE_TIME);

        if (useForceIndex && canStep && ballCount < MAX_BALLS) {
            if (forceIndexTimer.seconds() > FORCE_INDEX_INTERVAL) {
                ballCount++;

                if (ballCount >= MAX_BALLS) {
                    ballCount = MAX_BALLS;
                    intakeFull = true;
                    intakeIndex = MAX_BALLS - 1;
                } else {
                    intakeIndex = ballCount - 1;
                }

                sorterServo.setPosition(INTAKE_POS[intakeIndex]);
                servoMoving = true;
                servoMoveTimer.reset();
                forceIndexTimer.reset();
            }
        }

        boolean ballPresent = brightness > BALL_DETECT;
        boolean ballCleared = brightness < BALL_CLEAR;

        if (ballCleared || (ballDetected && ballTimer.seconds() > REARM_TIMEOUT)) {
            ballDetected = false;
        }

        if (ballPresent && !ballDetected && !launchMode && !intakeFull && !useForceIndex
                && ballTimer.seconds() > BALL_COOLDOWN
                && canStep) {

            ballDetected = true;
            ballTimer.reset();
            ballCount++;

            if (ballCount >= MAX_BALLS) {
                ballCount = MAX_BALLS;
                intakeFull = true;
                intakeIndex = MAX_BALLS - 1;
            } else {
                intakeIndex = ballCount - 1;
            }

            sorterServo.setPosition(INTAKE_POS[intakeIndex]);
            servoMoving = true;
            servoMoveTimer.reset();
        }
    }

    // ===== Basic actuator wrappers =====
    public void intake(double power) {
        intakeMotor.setPower(power);
    }

    public void aim(int targetTicks) {
        aimTargetTicks = targetTicks;
        aimHoldEnabled = true;

        aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        aimMotor.setTargetPosition(aimTargetTicks);
        aimMotor.setPower(AIM_OFFSET_POWER);
    }

    // ===== Launching =====
    public void startLaunch(int balls) {
        if (launching || balls <= 0) return;
        launching = true;
        launchMode = true;
    }

    public void setIntakeMode() {
        launchMode = false;
        sorterServo.setPosition(INTAKE_POS[intakeIndex]);
    }

    public void setLaunchMode() {
        launchMode = true;
        sorterServo.setPosition(LAUNCH_POS[intakeIndex]);
    }

    public void updateLauncher() {
        if (!launchMode) {
            launchState = LaunchState.IDLE;
            return;
        }

        switch (launchState) {
            case IDLE:
                sorterServo.setPosition(LAUNCH_POS[intakeIndex]);
                pushServo.setPosition(PUSH_DOWN_POS);
                launchState = LaunchState.ROTATE;
                break;

            case ROTATE:
                sorterServo.setPosition(LAUNCH_POS[intakeIndex]);
                launchTimer.reset();
                launchState = LaunchState.WAIT_SETTLE;
                break;

            case WAIT_SETTLE:
                if (launchTimer.seconds() > SETTLE_TIME) {
                    launchState = LaunchState.PUSH_UP;
                }
                break;

            case PUSH_UP:
                pushServo.setPosition(PUSH_UP_POS);
                launchTimer.reset();
                launchState = LaunchState.WAIT_UP;
                break;

            case WAIT_UP:
                if (launchTimer.seconds() > PUSH_UP_TIME) {
                    launchState = LaunchState.PUSH_DOWN;
                }
                break;

            case PUSH_DOWN:
                pushServo.setPosition(PUSH_DOWN_POS);
                launchTimer.reset();
                launchState = LaunchState.WAIT_DOWN;
                break;

            case WAIT_DOWN:
                if (launchTimer.seconds() > PUSH_DOWN_TIME) {
                    intakeIndex = (intakeIndex + 1) % LAUNCH_POS.length;
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    // ===== Path state machine (UPDATED PATH_1 behavior) =====
    public void setPathState(PathState newState) {
        pathState = newState;
        if (pathTimer != null) pathTimer.reset();
    }

    public void updateStateMachine() {
        switch (pathState) {

            case PATH_1:
                if (!initAimComplete) {
                    aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    // CHANGED: aim to your real tuned ticks
                    aim(PATH1_AIM_TICKS);

                    aimHoldEnabled = true;
                    initAimComplete = true;

                    // CHANGED: make sure shooter angle starts at max for this shot
                    shooterServo.setPosition(SERVO_MAX); // 0.45 [web:60]
                    path1SeqTimer.reset();
                    path1ServoWasSet = true;
                    path1ShooterStarted = false;

                    // CHANGED: fixed velocity mode so LL won't overwrite 6000 TPS
                    useFixedShooterVelocity = true;
                }

                if (!follower.isBusy()) {

                    // Step: after servo settle time, start shooter at 6000 TPS (ticks/sec) [web:36]
                    if (path1ServoWasSet && !path1ShooterStarted
                            && path1SeqTimer.seconds() >= PATH1_SERVO_SETTLE_SECONDS) {
                        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        shooter1.setVelocity(PATH1_SHOOTER_TARGET_TPS);
                        path1ShooterStarted = true;
                    }

                    if (!path3DelayStarted) {
                        path3Timer.reset();
                        path3DelayStarted = true;
                    }

                    // Start launching after 3 seconds (your old behavior)
                    if (path3Timer.seconds() >= 3.0) {
                        setLaunchMode();
                        startLaunch(3);
                    }

                    // During launch in Path1:
                    // - keep turret aiming with LL
                    // - keep shooterServo at max
                    // - keep shooter speed fixed at 6000 TPS
                    if (launchMode) {
                        shooterServo.setPosition(SERVO_MAX);
                        updateLimelightAimAndShooter(true, false);
                        shooter1.setVelocity(PATH1_SHOOTER_TARGET_TPS);
                    }

                    if (path3Timer.seconds() >= PATH1_DELAY_SECONDS) {
                        stopShooter();
                        resetBallState();

                        follower.followPath(paths.Path1, true);
                        follower.setMaxPower(1);
                        setPathState(PathState.PATH_2);

                        intake(1);
                        path3DelayStarted = false;
                    }
                }
                break;

            case PATH_2:
                if (!follower.isBusy()) {
                    setIntakeMode();
                    aim(0);

                    useForceIndex = true;
                    forceIndexTimer.reset();

                    follower.followPath(paths.Path2, true);
                    intake(1);
                    follower.setMaxPower(0.4);
                    setPathState(USE_PATH3 ? PathState.PATH_3 : PathState.PATH_4);
                }
                break;

            case PATH_3:
                if (!follower.isBusy()) {
                    useForceIndex = false;
                    resetBallState();
                    intake(0);

                    setShooterRPM(1400);

                    follower.followPath(paths.Path3, true);
                    setPathState(PathState.PATH_4);
                    follower.setMaxPower(1);

                    aim(45);
                }
                break;

            case PATH_4:
                if (!follower.isBusy()) {
                    setLaunchMode();
                    startLaunch(3);

                    if (!path3DelayStarted) {
                        path3Timer.reset();
                        path3DelayStarted = true;
                    }

                    if (!useFixedShooterVelocity) updateLimelightAimAndShooter(true, true);

                    if (path3Timer.seconds() >= PATH3_DELAY_SECONDS) {
                        resetBallState();
                        follower.followPath(paths.Path4, true);
                        setPathState(PathState.PATH_5);

                        stopShooter();

                        follower.setMaxPower(1);
                        path3DelayStarted = false;
                    }
                }
                break;

            case PATH_5:
                if (!follower.isBusy()) {
                    setIntakeMode();

                    useForceIndex = true;
                    forceIndexTimer.reset();

                    follower.followPath(paths.Path5, true);
                    follower.setMaxPower(0.4);
                    setPathState(PathState.PATH_6);

                    intake(1);
                    aim(0);
                }
                break;

            case PATH_6:
                if (!follower.isBusy()) {
                    useForceIndex = false;

                    follower.followPath(paths.Path6, true);
                    setPathState(PathState.PATH_7);

                    setShooterRPM(300);
                    useFixedShooterVelocity = true;

                    follower.setMaxPower(1);
                    aim(-70);
                }
                break;

            case PATH_7:
                if (!follower.isBusy()) {
                    setLaunchMode();
                    startLaunch(3);

                    if (!path3DelayStarted) {
                        path3Timer.reset();
                        path3DelayStarted = true;
                    }

                    if (useFixedShooterVelocity) {
                        updateLimelightAimAndShooter(true, false);
                        shooter1.setVelocity(rpmToTicksPerSecond(targetShooterRPM));
                    } else {
                        updateLimelightAimAndShooter(true, true);
                    }

                    if (path3Timer.seconds() >= PATH3_DELAY_SECONDS) {
                        resetBallState();
                        setIntakeMode();

                        useFixedShooterVelocity = false;

                        useForceIndex = true;
                        forceIndexTimer.reset();

                        follower.followPath(paths.Path7, true);
                        intake(0);
                        follower.setMaxPower(0.4);
                        setPathState(PathState.PATH_8);

                        path3DelayStarted = false;
                    }
                }
                break;

            case PATH_8:
                if (!follower.isBusy()) {
                    useForceIndex = false;
                    resetBallState();
                    intake(0);

                    setShooterRPM(1200);

                    follower.followPath(paths.Path8, true);
                    setPathState(PathState.PATH_9);
                    follower.setMaxPower(1);
                }
                break;

            case PATH_9:
                if (!follower.isBusy()) {
                    setLaunchMode();
                    startLaunch(3);

                    if (!path3DelayStarted) {
                        path3Timer.reset();
                        path3DelayStarted = true;
                    }

                    if (!useFixedShooterVelocity) updateLimelightAimAndShooter(true, true);

                    if (path3Timer.seconds() >= PATH3_DELAY_SECONDS) {
                        follower.followPath(paths.Path9, true);
                        setPathState(PathState.DONE);

                        stopShooter();

                        follower.setMaxPower(1);
                        path3DelayStarted = false;
                    }
                }
                break;

            case DONE:
                if (!follower.isBusy()) telemetry.addLine("AUTO COMPLETE ✔");
                break;
        }
    }

    // ===== FTC OpMode lifecycle =====
    @Override
    public void init() {
        try {
            pushServo = hardwareMap.get(Servo.class, "pushservo");
            pushServo.setPosition(PUSH_HOME);

            shooterServo = hardwareMap.get(Servo.class, "shooterServo");
            shooterServo.setPosition(SERVO_MIN);

            sorterServo = hardwareMap.get(Servo.class, "sortservo");
            sorterServo.setPosition(INTAKE_POS[0]);

            colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");

            shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
            shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

            PIDFCoefficients pidf = new PIDFCoefficients(31, 0, 0, 23);
            shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

            shooterTicksPerRev = shooter1.getMotorType().getTicksPerRev();

            intakeMotor = hardwareMap.get(DcMotor.class, "intake");

            aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
            aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            aimMotor.setDirection(DcMotor.Direction.REVERSE);
            aimMotor.setPower(0);

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(2);
            limelight.setPollRateHz(100);
            limelight.start();

            pathTimer = new ElapsedTime();
            opModeTimer = new ElapsedTime();

            follower = Constants.createFollower(hardwareMap);
            follower.setPose(new Pose(59.0, 15.0, Math.toRadians(180)));

            // CHANGED: Paths needs opMode for callbacks
            paths = new Paths(follower, this);

            telemetry.addLine("✅ 🔵 BLUE AUTO (TeleOp Shooter+LL)");
            telemetry.addData("Path1 AimTicks", PATH1_AIM_TICKS);
            telemetry.addData("Path1 ShooterTPS", PATH1_SHOOTER_TARGET_TPS);
            telemetry.addData("Path1 ServoMax", SERVO_MAX);
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("❌ Init Error: " + e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void start() {
        opModeTimer.reset();
        setPathState(PathState.PATH_1);

        ballCount = 3;
        intakeIndex = 2;
        ballDetected = false;
        intakeFull = true;

        initAimComplete = false;
        filteredTx = 0.0;

        ballTimer.reset();
        servoMoveTimer.reset();
        forceIndexTimer.reset();

        path3DelayStarted = false;
        useFixedShooterVelocity = false;

        // reset Path1 sequence flags
        path1SeqTimer.reset();
        path1ServoWasSet = false;
        path1ShooterStarted = false;

        launching = false;
        launchMode = false;
        launchState = LaunchState.IDLE;
    }

    @Override
    public void loop() {
        try {
            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();
            int brightness = r + g + b;

            if (!launchMode && !servoMoving) {
                updateBallSensor(brightness);
            }

            if (servoMoving && servoMoveTimer.seconds() > SERVO_SETTLE_TIME) {
                servoMoving = false;
            }

            if (launchMode) {
                sorterServo.setPosition(LAUNCH_POS[intakeIndex]);

                // CHANGED: if fixed mode (Path1), do NOT overwrite shooter speed; keep 6000 TPS
                if (useFixedShooterVelocity && pathState == PathState.PATH_1) {
                    updateLimelightAimAndShooter(true, false);
                    shooterServo.setPosition(SERVO_MAX);
                    shooter1.setVelocity(PATH1_SHOOTER_TARGET_TPS);
                } else if (useFixedShooterVelocity) {
                    updateLimelightAimAndShooter(true, false);
                    shooter1.setVelocity(rpmToTicksPerSecond(targetShooterRPM));
                } else {
                    updateLimelightAimAndShooter(true, true);
                }
            }

            if (aimHoldEnabled) {
                if (aimMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    aimMotor.setTargetPosition(aimTargetTicks);
                    aimMotor.setPower(AIM_OFFSET_POWER);
                }
            }

            follower.update();
            updateStateMachine();
            updateLauncher();

            telemetry.addData("State", pathState);
            telemetry.addData("Balls", ballCount);
            telemetry.addData("Idx", intakeIndex);
            telemetry.addData("Bright", brightness);

            telemetry.addData("AimPos", aimMotor.getCurrentPosition());
            telemetry.addData("AimTarget", aimTargetTicks);

            telemetry.addData("Path1 TPS target", (int) PATH1_SHOOTER_TARGET_TPS);
            telemetry.addData("Actual TPS", (int) shooter1.getVelocity()); // ticks/sec [web:36]
            telemetry.addData("Servo Pos", String.format("%.3f", shooterServo.getPosition()));
            telemetry.addData("Fixed Mode", useFixedShooterVelocity);

            LLResult res = limelight.getLatestResult();
            if (res != null && res.isValid()) {
                telemetry.addData("LL tx", String.format("%.2f", res.getTx()));
                telemetry.addData("LL ty", String.format("%.2f", res.getTy()));
                telemetry.addData("LL ta", String.format("%.2f", res.getTa()));
                telemetry.addData("LL stale(ms)", res.getStaleness());
                telemetry.addData("LL pipe", res.getPipelineIndex());
            } else {
                telemetry.addData("LL", "No/Invalid");
            }

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
    }
}