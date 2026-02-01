package org.firstinspires.ftc.teamcode.code;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled

@TeleOp(name = "Turret + Shooter ")
public class code1 extends OpMode {

    private Follower follower;
    private DcMotor aimMotor;
    private DcMotorEx shooter1;
    private DcMotor intake, intake2;
    private Servo shooterServo;

    // --- COORDINATES ---
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;
    private static final Pose START_POSE = new Pose(56, 8, Math.toRadians(90));
    private static final double HEADING_OFFSET = Math.toRadians(-90);

    // --- TURRET ---
    private static final double TICKS_PER_REV = 1627;
    private static final double AIM_TICKS_PER_RAD = TICKS_PER_REV / (2.0 * Math.PI);
    private static final int AIM_MIN_TICKS = -2000;
    private static final int AIM_MAX_TICKS = 2000;
    private static final double AIM_POWER = 0.6;
    private static final double LOOKAHEAD_TIME = 0.15;

    // --- SHOOTER PHYSICS ---
    private static final double WHEEL_RADIUS_IN = (9.0 / 2.54) / 2.0;
    private static final double GRAVITY = 386.09;

    // PRECISION TUNING
    private static final double SLIP_EFFICIENCY = 0.70;

    // MAGNUS EFFECT TUNING
    // 0.0 = No Lift. 0.35 = Standard Foam Ball. 0.6 = Plastic Ball.
    // If shots float too much (overshoot), LOWER this.
    private static final double LIFT_COEFFICIENT = 0.35;

    // HEIGHTS
    private static final double SHOOTER_HEIGHT_IN = 45.0 / 2.54;
    private static final double GOAL_HEIGHT_IN = 98.45 / 2.54;
    private static final double HEIGHT_DIFF = GOAL_HEIGHT_IN - SHOOTER_HEIGHT_IN;

    // BALL SPECS & DRAG
    private static final double BALL_MASS_LB = 0.165;
    private static final double BALL_DIAMETER_IN = 5.0;
    private static final double BALL_RADIUS_IN = BALL_DIAMETER_IN / 2.0;
    private static final double BALL_CROSS_SECTION = Math.PI * BALL_RADIUS_IN * BALL_RADIUS_IN;
    private static final double AIR_DENSITY_SLUG_IN3 = 0.0023769;
    private static final double DRAG_COEFFICIENT = 0.47;
    private static final double MASS_SLUG = BALL_MASS_LB / 32.174;

    // Pre-calculated constants
    // Drag K: F_drag = K * v^2
    private static final double DRAG_K = (0.5 * DRAG_COEFFICIENT * AIR_DENSITY_SLUG_IN3 * BALL_CROSS_SECTION) / MASS_SLUG;
    // Lift K: F_lift = K * v^2
    private static final double LIFT_K = (0.5 * LIFT_COEFFICIENT * AIR_DENSITY_SLUG_IN3 * BALL_CROSS_SECTION) / MASS_SLUG;

    // MOTOR
    private static final double TICKS_PER_SHOOTER_REV = 28.0;
    private static final double MAX_RPM = 6000.0;
    public static double SHOOTER_P = 15.0;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 13.0;

    // HOOD
    private static final double HOOD_MIN_ANGLE = 25.0;
    private static final double HOOD_MAX_ANGLE = 45.0;
    private static final double SERVO_MAX_POS = 0.50;

    // STATE
    private boolean turretLock = false;
    private boolean dpadUpLatch = false;
    private boolean shooterActive = false;
    private boolean xLatch = false;

    private double lastX=0, lastY=0, lastHeading=0;
    private long lastTimeNs=0;
    private double angularVel=0, velX=0, velY=0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aimMotor.setTargetPosition(0);
        aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        aimMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        aimMotor.setPower(AIM_POWER);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);



        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterServo.setPosition(0.0);

        follower.startTeleopDrive();
    }

    @Override
    public void start() {
        Pose p = follower.getPose();
        lastX=p.getX(); lastY=p.getY(); lastHeading=p.getHeading();
        lastTimeNs=System.nanoTime();
    }

    @Override
    public void loop() {
        follower.update();

        long now = System.nanoTime();
        Pose currentPose = follower.getPose();
        double dt = (now - lastTimeNs) / 1e9;
        if (dt > 0.005) {
            velX = (currentPose.getX() - lastX) / dt;
            velY = (currentPose.getY() - lastY) / dt;
            angularVel = AngleUnit.normalizeRadians(currentPose.getHeading() - lastHeading) / dt;
            lastX=currentPose.getX(); lastY=currentPose.getY(); lastHeading=currentPose.getHeading(); lastTimeNs=now;
        }

        if (gamepad1.dpad_up && !dpadUpLatch) { dpadUpLatch = true; turretLock = !turretLock; }
        if (!gamepad1.dpad_up) dpadUpLatch = false;

        if (gamepad1.x && !xLatch) { xLatch = true; shooterActive = !shooterActive; }
        if (!gamepad1.x) xLatch = false;

        if (gamepad1.left_trigger > 0.1) { intake.setPower(1); intake2.setPower(0); }
        else if (gamepad1.right_trigger > 0.1) { intake.setPower(1); intake2.setPower(1); }
        else { intake.setPower(0); intake2.setPower(0); }

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        if (turretLock) {
            Pose p = follower.getPose();
            double predX = clamp(p.getX() + (velX * LOOKAHEAD_TIME), 0, 144);
            double predY = clamp(p.getY() + (velY * LOOKAHEAD_TIME), 0, 144);
            double predH = p.getHeading() + (angularVel * LOOKAHEAD_TIME);

            double dx = GOAL_X - predX;
            double dy = GOAL_Y - predY;
            double distToGoal = Math.hypot(dx, dy);

            // Turret Aim
            double goalAngle = Math.atan2(dy, dx) + HEADING_OFFSET;
            double relativeRad = AngleUnit.normalizeRadians(goalAngle - predH);
            int targetTicks = clampInt((int)Math.round(relativeRad * AIM_TICKS_PER_RAD), AIM_MIN_TICKS, AIM_MAX_TICKS);
            aimMotor.setTargetPosition(targetTicks);

            if (shooterActive) {
                // 1. Solve (Magnus + Drag)
                ShooterSolution sol = solveShooterMagnus(distToGoal, HEIGHT_DIFF);
                shooterServo.setPosition(sol.servoPos);

                if (sol.valid) {
                    // 2. Motion Comp
                    double robotVelTowardsGoal = (velX * dx + velY * dy) / distToGoal;
                    double adjustedExitVel = sol.exitVel - robotVelTowardsGoal;
                    double finalRPM = velocityToRPM(adjustedExitVel);

                    double tps = (finalRPM / 60.0) * TICKS_PER_SHOOTER_REV;
                    shooter1.setVelocity(tps);

                    double currentRPM = (shooter1.getVelocity() / TICKS_PER_SHOOTER_REV) * 60.0;
                    boolean isReady = Math.abs(finalRPM - currentRPM) < 150;

                    telemetry.addData("STATUS", isReady ? ">>> FIRE! <<<" : "Spinning...");
                    telemetry.addData("Dist", "%.1f", distToGoal);
                    telemetry.addData("Angle", "%.1f", sol.angleDeg);
                    telemetry.addData("RPM", "%.0f", finalRPM);
                } else {
                    shooter1.setPower(0);
                    telemetry.addData("STATUS", "No Solution");
                }
            } else {
                shooter1.setPower(0);
            }
        } else {
            if (shooterActive) shooter1.setPower(0.3); else shooter1.setPower(0);
        }

        telemetry.addData("Lock", turretLock);
        telemetry.update();
    }

    // --- MAGNUS SOLVER ---
    private static class ShooterSolution {
        double angleDeg, rpm, servoPos, exitVel;
        boolean valid = false;
    }

    private ShooterSolution solveShooterMagnus(double targetDist, double targetHeight) {
        ShooterSolution best = new ShooterSolution();

        // Scan angles
        for (double angle = HOOD_MIN_ANGLE; angle <= HOOD_MAX_ANGLE; angle += 1.0) {
            double minV = 100, maxV = 600;
            for (int iter = 0; iter < 12; iter++) { // 12 iters for precision
                double testV = (minV + maxV) / 2.0;

                // Simulate with Lift + Drag
                double landDist = simulateMagnusTrajectory(testV, angle, targetHeight);

                if (Math.abs(landDist - targetDist) < 1.0) {
                    double rpm = velocityToRPM(testV);
                    if (rpm <= MAX_RPM) {
                        if (!best.valid || rpm < best.rpm) {
                            best.valid = true;
                            best.angleDeg = angle;
                            best.rpm = rpm;
                            best.exitVel = testV;
                            best.servoPos = angleToServo(angle);
                        }
                    }
                    break;
                }
                if (landDist < targetDist) minV = testV; else maxV = testV;
            }
        }

        if (!best.valid) {
            best.angleDeg = HOOD_MAX_ANGLE;
            best.rpm = MAX_RPM;
            best.exitVel = 600;
            best.servoPos = angleToServo(HOOD_MAX_ANGLE);
        }
        return best;
    }

    private double simulateMagnusTrajectory(double v0, double angleDeg, double targetHeight) {
        double theta = Math.toRadians(angleDeg);
        double vx = v0 * Math.cos(theta);
        double vy = v0 * Math.sin(theta);
        double x = 0, y = 0, dt = 0.002;

        // Magnus force direction is perpendicular to velocity
        // If Velocity is (vx, vy), Lift is (-vy, vx) * K (Assuming Backspin)

        boolean passedApex = false;

        for (int step = 0; step < 5000; step++) {
            double v = Math.sqrt(vx*vx + vy*vy);

            // Drag Force (Opposite to velocity)
            double fDrag = DRAG_K * v * v;
            double axDrag = -fDrag * (vx / v);
            double ayDrag = -fDrag * (vy / v);

            // Lift Force (Perpendicular to velocity)
            // Backspin creates lift UPWARD relative to velocity vector
            double fLift = LIFT_K * v * v;
            double axLift = -fLift * (vy / v); // Rotated 90 deg
            double ayLift = fLift * (vx / v);

            // Net Accel
            double ax = axDrag + axLift;
            double ay = -GRAVITY + ayDrag + ayLift;

            vx += ax * dt; vy += ay * dt; x += vx * dt; y += vy * dt;

            if (vy < 0) passedApex = true;
            if (passedApex && y <= targetHeight) return x;
        }
        return x;
    }

    private double velocityToRPM(double linearVel) {
        double wheelSpeedLin = linearVel / SLIP_EFFICIENCY;
        double circum = 2.0 * Math.PI * WHEEL_RADIUS_IN;
        return (wheelSpeedLin * 60.0) / circum;
    }

    private double angleToServo(double angle) {
        double clamped = clamp(angle, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
        return ((clamped - HOOD_MIN_ANGLE) / (HOOD_MAX_ANGLE - HOOD_MIN_ANGLE)) * SERVO_MAX_POS;
    }

    private static int clampInt(int v, int lo, int hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}