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
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled

@TeleOp(name = "Turret + Shooter (Original Coords)")
public class lat extends OpMode {

    private Follower follower;
    private DcMotor aimMotor;
    private DcMotorEx shooter1;
    private Servo shooterServo;

    // --- REVERTED COORDINATES (v7/vFinal values) ---
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;
    private static final Pose START_POSE = new Pose(56, 8, Math.toRadians(90));
    private static final double HEADING_OFFSET = Math.toRadians(-90);

    // Turret
    private static final double TICKS_PER_REV = 1627;
    private static final double AIM_TICKS_PER_RAD = TICKS_PER_REV / (2.0 * Math.PI);
    private static final int AIM_MIN_TICKS = -2000;
    private static final int AIM_MAX_TICKS = 2000;
    private static final double AIM_POWER = 0.6;
    private static final double LOOKAHEAD_TIME = 0.15;

    // Shooter Physics
    private static final double SHOOTER_HEIGHT_INCHES = 14.0;
    private static final double GOAL_HEIGHT_INCHES = 33.0;
    private static final double HEIGHT_DIFF = GOAL_HEIGHT_INCHES - SHOOTER_HEIGHT_INCHES;
    private static final double TICKS_PER_SHOOTER_REV = 28.0;
    private static final double GRAVITY = 386.0;
    private static final double EXIT_VEL_ADJUST = 1.15;
    private static final double MAX_RPM = 5500.0;

    // Hood Servo (0.0 to 0.50)
    private static final double HOOD_MIN_ANGLE = 25.0;
    private static final double HOOD_MAX_ANGLE = 45.0;
    private static final double SERVO_MAX_POS = 0.50;

    private boolean turretLock = false;
    private boolean dpadUpLatch = false;
    private boolean shooterActive = false;
    private boolean xLatch = false;

    // Velocity Manual Calc
    private double lastX = 0.0, lastY = 0.0, lastHeading = 0.0;
    private long lastTimeNs = 0;
    private double angularVel = 0.0, velX = 0.0, velY = 0.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        // Turret
        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aimMotor.setTargetPosition(0);
        aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        aimMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        aimMotor.setPower(AIM_POWER);

        // Shooter
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterServo.setPosition(0.0);

        follower.startTeleopDrive();
    }

    @Override
    public void start() {
        Pose p = follower.getPose();
        lastX = p.getX(); lastY = p.getY(); lastHeading = p.getHeading();
        lastTimeNs = System.nanoTime();
    }

    @Override
    public void loop() {
        follower.update();

        // 1. Velocity Calc
        long currentTimeNs = System.nanoTime();
        Pose currentPose = follower.getPose();
        double dt = (currentTimeNs - lastTimeNs) / 1e9;

        if (dt > 0.005) {
            velX = (currentPose.getX() - lastX) / dt;
            velY = (currentPose.getY() - lastY) / dt;
            angularVel = AngleUnit.normalizeRadians(currentPose.getHeading() - lastHeading) / dt;

            lastX = currentPose.getX(); lastY = currentPose.getY(); lastHeading = currentPose.getHeading();
            lastTimeNs = currentTimeNs;
        }

        // Toggle Lock
        if (gamepad1.dpad_up && !dpadUpLatch) {
            dpadUpLatch = true;
            turretLock = !turretLock;
        }
        if (!gamepad1.dpad_up) dpadUpLatch = false;

        // Toggle Shooter
        if (gamepad1.x && !xLatch) {
            xLatch = true;
            shooterActive = !shooterActive;
        }
        if (!gamepad1.x) xLatch = false;

        // Drive
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        // AIMING
        if (turretLock) {
            Pose p = follower.getPose();

            // Predict Future Pose
            double predX = p.getX() + (velX * LOOKAHEAD_TIME);
            double predY = p.getY() + (velY * LOOKAHEAD_TIME);
            double predH = p.getHeading() + (angularVel * LOOKAHEAD_TIME);

            // Vector to Goal
            double dx = GOAL_X - predX;
            double dy = GOAL_Y - predY;
            double distToGoal = Math.hypot(dx, dy);

            // Turret Aim
            double goalAngle = Math.atan2(dy, dx) + HEADING_OFFSET;
            double relativeRad = AngleUnit.normalizeRadians(goalAngle - predH);
            int targetTicks = (int) Math.round(relativeRad * AIM_TICKS_PER_RAD);
            targetTicks = clampInt(targetTicks, AIM_MIN_TICKS, AIM_MAX_TICKS);
            aimMotor.setTargetPosition(targetTicks);

            // Shooter Ballistics
            if (shooterActive) {
                double shotAngle = 45.0;
                shooterServo.setPosition(angleToServo(shotAngle));

                double targetRPM = calculateRPM(distToGoal, HEIGHT_DIFF, shotAngle);

                if (targetRPM > 0) {
                    double tps = (targetRPM / 60.0) * TICKS_PER_SHOOTER_REV;
                    shooter1.setVelocity(tps);
                } else {
                    shooter1.setPower(0);
                }
            } else {
                shooter1.setPower(0);
            }
        } else {
            if (shooterActive) shooter1.setPower(0.5);
            else shooter1.setPower(0);
        }

        telemetry.addData("TurretLock", turretLock);
        telemetry.addData("Shooter", shooterActive ? "ON" : "OFF");
        if (shooterActive && turretLock) {
            telemetry.addData("Target RPM", "%.0f", shooter1.getVelocity() / TICKS_PER_SHOOTER_REV * 60.0);
        }
        telemetry.update();
    }

    private double calculateRPM(double distance, double heightDiff, double angleDeg) {
        double theta = Math.toRadians(angleDeg);
        double cos = Math.cos(theta);
        double tan = Math.tan(theta);
        double denom = 2 * Math.pow(cos, 2) * (distance * tan - heightDiff);

        if (denom <= 0) return 0;

        double v = Math.sqrt((GRAVITY * Math.pow(distance, 2)) / denom) * EXIT_VEL_ADJUST;
        double wheelCircumference = 2 * Math.PI * 2.0;
        return Math.min((v * 60.0) / wheelCircumference, MAX_RPM);
    }

    private double angleToServo(double angle) {
        double clamped = Math.max(HOOD_MIN_ANGLE, Math.min(HOOD_MAX_ANGLE, angle));
        double pct = (clamped - HOOD_MIN_ANGLE) / (HOOD_MAX_ANGLE - HOOD_MIN_ANGLE);
        return pct * SERVO_MAX_POS;
    }

    private static int clampInt(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}