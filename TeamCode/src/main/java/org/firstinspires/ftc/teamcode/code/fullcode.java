package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
@Disabled

@TeleOp(name = "BLUE1-1")
public class fullcode extends OpMode {
    GoBildaPinpointDriver pinpoint;

    // --- SHOOTER & INTAKE VARIABLES ---
    DcMotorEx shooter;
    double shooterSpeed = 0.0;
    DcMotor intake;
    DcMotor intake2;
    Servo angleServo;
    double anglePos = 0;

    // --- TURRET VARIABLES (From track2) ---
    DcMotor aimMotor;
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;
    private static final double HEADING_OFFSET = Math.toRadians(-144); // Adjust if your turret zero differs

    private static final double TICKS_PER_REV = 1627;
    private static final double AIM_TICKS_PER_RAD = TICKS_PER_REV / (2.0 * Math.PI);
    private static final int AIM_MIN_TICKS = -2000;
    private static final int AIM_MAX_TICKS = 2000;
    private static final double AIM_POWER = 0.6;

    boolean turretLock = false;
    boolean dpadUpLatch = false;

    // --- LOCALIZATION VARS ---
    public double currentHeading, currentX, currentY, distance, deltaY, deltaX, velocityX, velocityY, relative_angle;

    @Override
    public void init() {
        // --- HARDWARE MAPPING ---
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        angleServo = hardwareMap.get(Servo.class, "shooterServo");

        // Added Aim Motor
        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");

        // --- MOTOR CONFIGURATION ---
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Turret Configuration (From track2)
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aimMotor.setTargetPosition(0);
        aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        aimMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        aimMotor.setPower(AIM_POWER);

        // --- PINPOINT SETUP ---
        pinpoint.setOffsets(5.14, -3.918, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // Starting Pose
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 27, 130, AngleUnit.DEGREES, 144));

        // Start Intakes & Servo
        intake.setPower(1);
        intake2.setPower(1);
        angleServo.setPosition(anglePos);
    }

    @Override
    public void loop() {
        // 1. Update Localization
        update_blue();

        // 2. Shooter Logic (Existing)
        shooterSpeed = 6.85 * distance + 1050;
        shooter.setVelocity(shooterSpeed);

        // Manual Adjust
        if (gamepad1.dpadLeftWasPressed()) shooterSpeed += 10;
        if (gamepad1.dpadRightWasPressed()) shooterSpeed -= 10;

        // 3. Servo Angle Logic (Existing)
        if (distance < 55) anglePos = 0.00;
        else if (distance < 65) anglePos = 0.01;
        else if (distance < 85) anglePos = 0.02;
        else if (distance < 110) anglePos = 0.03;
        else if (distance < 125) anglePos = 0.04;
        else anglePos = 0.06;
        angleServo.setPosition(anglePos);

        // 4. Turret Logic (Imported from track2)
        if (gamepad1.dpad_up && !dpadUpLatch) {
            dpadUpLatch = true;
            turretLock = !turretLock;
        }
        if (!gamepad1.dpad_up) dpadUpLatch = false;

        if (turretLock) {
            // Calculate vector to goal
            // Uses currentX/Y from update_blue()
            double dx = GOAL_X - currentX;
            double dy = GOAL_Y - currentY;

            // Math to calculate turret angle
            // Note: currentHeading is in Degrees, but atan2 returns Radians.
            // We convert currentHeading to Radians for the calculation.
            double currentHeadingRad = Math.toRadians(currentHeading);

            double goalFieldHeading = Math.atan2(dy, dx) + HEADING_OFFSET;
            double desiredTurretRad = AngleUnit.normalizeRadians(goalFieldHeading - currentHeadingRad);

            // Convert to Ticks and Clamp
            int targetTicks = (int) Math.round(desiredTurretRad * AIM_TICKS_PER_RAD);
            targetTicks = clampInt(targetTicks, AIM_MIN_TICKS, AIM_MAX_TICKS);

            aimMotor.setTargetPosition(targetTicks);
        }

        // 5. Telemetry
        telemetry.addData("--- SYSTEM STATUS ---", "");
        telemetry.addData("Turret Mode", turretLock ? "LOCKED" : "MANUAL");
        telemetry.addData("Pos X", "%.1f", currentX);
        telemetry.addData("Pos Y", "%.1f", currentY);
        telemetry.addData("Heading", "%.1f", currentHeading);
        telemetry.addData("Dist", "%.1f", distance);
        telemetry.addData("Shooter Vel", "%.0f", shooter.getVelocity());
        telemetry.update();
    }

    public void update_blue() {
        pinpoint.update();
        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        currentX = pinpoint.getPosX(DistanceUnit.INCH);
        currentY = pinpoint.getPosY(DistanceUnit.INCH);
        velocityX = pinpoint.getVelX(DistanceUnit.INCH);
        velocityY = pinpoint.getVelY(DistanceUnit.INCH);
        deltaY = 144 - currentY;
        deltaX = 0 - currentX;
        relative_angle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    // Helper function from track2
    private static int clampInt(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}