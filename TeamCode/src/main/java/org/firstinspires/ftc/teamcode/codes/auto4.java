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
public class auto4 extends OpMode {

    // ===== Hardware =====
    private Servo pushServo;
    private Servo sorterServo;
    private RevColorSensorV3 colorSensor;
    private DcMotorEx shooter1;
    private DcMotor aimMotor;
    private DcMotor intakeMotor;
    private Servo shooterServo;
    private Limelight3A limelight;

    // ===== Sorter positions (keep yours) =====PID
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

    // Limelight confidence/stability (you already had this style)
    static final long MAX_STALE_MS = 80;
    static final double TA_MIN = 0.15;

    // ===== Distance model + mapping (TeleOp values) =====
    // Use your TeleOp constants so Auto matches TeleOp behavior.
    private static final double CAMERA_HEIGHT = 0.25;   // meters
    private static final double TARGET_HEIGHT = 0.90;   // meters
    private static final double CAMERA_ANGLE  = 20.0;   // degrees

    private static final double SERVO_MIN = 0.30;
    private static final double SERVO_MAX = 0.45;//private static final double SERVO_MAX = 0.50;
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

    // ===== Paths (unchanged) =====
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

        public Paths(Follower follower) {

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

    // FTC SDK: setVelocity(double) is ticks/second. [page:1]
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * shooterTicksPerRev;
    }

    private void setShooterRPM(double rpm) {
        targetShooterRPM = rpm;
        shooter1.setVelocity(rpmToTicksPerSecond(rpm)); // ticks/sec [page:1]
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

    /**
     * TeleOp port:
     * - tx filtered -> turret power
     * - ty -> distance -> shooter servo + rpm
     *
     * Uses staleness + ta gating so Auto doesn't chase noise. [page:0]
     */
    private void updateLimelightAimAndShooter(boolean enableTurretAim, boolean enableShooter) {
        LLResult result = limelight.getLatestResult(); // LLResult usage [page:0]
        if (result == null || !result.isValid()) return; // LLResult validity pattern [page:0]

        if (result.getStaleness() > MAX_STALE_MS) return; // data freshness API [page:0]
        if (result.getTa() < TA_MIN) return;              // target area API [page:0]

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
                    // advance to next ball slot
                    intakeIndex = (intakeIndex + 1) % LAUNCH_POS.length;
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    // ===== Path state machine (your original structure; updated shooter+LL calls) =====
    public void setPathState(PathState newState) {
        pathState = newState;
        if (pathTimer != null) pathTimer.reset();
    }

    public void updateStateMachine() {
        switch (pathState) {

            case PATH_1:
                if (!initAimComplete) {
                    aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    // your mirror setting
                    aimTargetTicks = -322;

                    aimMotor.setTargetPosition(aimTargetTicks);
                    aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    aimMotor.setPower(AIM_OFFSET_POWER);
                    aimHoldEnabled = true;
                    initAimComplete = true;
                }

                if (!follower.isBusy()) {
                    // Start shooter early then launch after delay
                    setShooterRPM(RPM_MAX);

                    if (!path3DelayStarted) {
                        path3Timer.reset();
                        path3DelayStarted = true;
                    }

                    // start launching earlier
                    if (path3Timer.seconds() >= 3.0) {
                        setLaunchMode();
                        startLaunch(3);
                    }

                    // during launch, keep LL aiming + servo/rpm updating
                    if (launchMode) {
                        if (!useFixedShooterVelocity) updateLimelightAimAndShooter(true, true);
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

                    // Aim + auto tune while launching
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

                    // Fixed shot segment like your old code
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

                    // In fixed mode, keep rpm constant but still allow turret aim + servo angle
                    if (useFixedShooterVelocity) {
                        updateLimelightAimAndShooter(true, false);
                        shooter1.setVelocity(rpmToTicksPerSecond(targetShooterRPM)); // ticks/sec [page:1]
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

            // Keep your PIDF; tune as needed.
            PIDFCoefficients pidf = new PIDFCoefficients(31, 0, 0, 23);//(5, 0, 0, 19);
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
            limelight.start(); // start polling [page:0]

            pathTimer = new ElapsedTime();
            opModeTimer = new ElapsedTime();

            follower = Constants.createFollower(hardwareMap);

            follower.setPose(new Pose(59.0, 15.0, Math.toRadians(180)));

            paths = new Paths(follower);

            telemetry.addLine("✅ 🔵 BLUE AUTO (TeleOp Shooter+LL)");
            telemetry.addData("Pipeline", 2);
            telemetry.addData("Shooter ticks/rev", shooterTicksPerRev);
            telemetry.addLine("Note: setVelocity() uses ticks/sec (RPM converted)."); // [page:1]
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

            // If launching, keep TeleOp-style LL updates alive continuously
            if (launchMode) {
                sorterServo.setPosition(LAUNCH_POS[intakeIndex]);

                if (useFixedShooterVelocity) {
                    updateLimelightAimAndShooter(true, false);
                    shooter1.setVelocity(rpmToTicksPerSecond(targetShooterRPM)); // ticks/sec [page:1]
                } else {
                    updateLimelightAimAndShooter(true, true);
                }
            }

            // Keep RUN_TO_POSITION hold alive
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
            telemetry.addData("txFiltered", String.format("%.2f", filteredTx));

            telemetry.addData("Shooter ticks/rev", shooterTicksPerRev);
            telemetry.addData("Target RPM", (int) targetShooterRPM);
            telemetry.addData("Target TPS", (int) rpmToTicksPerSecond(targetShooterRPM));
            telemetry.addData("Actual TPS", (int) shooter1.getVelocity()); // ticks/sec [page:1]
            telemetry.addData("Servo Pos", String.format("%.3f", shooterServo.getPosition()));
            telemetry.addData("Fixed Mode", useFixedShooterVelocity);

            LLResult res = limelight.getLatestResult(); // LLResult usage [page:0]
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
