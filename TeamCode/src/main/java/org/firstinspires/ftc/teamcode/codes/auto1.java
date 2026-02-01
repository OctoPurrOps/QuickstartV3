package org.firstinspires.ftc.teamcode.codes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
@Disabled
@TeleOp(name="ShooterAimTuner", group="Tuning")
public class auto1 extends LinearOpMode {

    DcMotorEx shooter;
    DcMotorEx turret;

    // drivetrain motors (rename to match your configuration)
    DcMotor frontLeft, frontRight, backLeft, backRight;

    // intake motor (rename to match your configuration)
    DcMotor intake;

    // Shooter target (ticks per second)
    public static double targetTPS = 0;
    public static double stepTPS = 200;
    public static double maxTPS = 6000;

    // Turret target (encoder ticks)
    public static int turretTargetTicks = 0;
    public static int turretStepTicks = 10;

    // edge detect
    boolean prevDpadUp = false;
    boolean prevDpadDown = false;
    boolean prevY = false;
    boolean prevA = false;
    boolean prevX = false;

    @Override
    public void runOpMode() {

        shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        turret  = hardwareMap.get(DcMotorEx.class, "aimMotor");

        // NEW: map drivetrain motors
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // NEW: map intake motor
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Shooter setup
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // MAKE SHOOTER RUN IN REVERSE
        shooter.setDirection(DcMotor.Direction.REVERSE); // <--- added (reverse shooter) [web:14][web:16]

        // Turret setup
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Typical mecanum direction setup; flip as needed for your robot
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            // ================== MECANUM DRIVE ==================
            // gamepad1:
            // left stick: drive (y) + strafe (x)
            // right stick x: rotation
            double drive  = -gamepad1.left_stick_y;   // forward/back
            double strafe =  gamepad1.left_stick_x;   // left/right
            double turn   =  gamepad1.right_stick_x;  // rotate

            double fl = drive + strafe + turn;
            double fr = drive - strafe - turn;
            double bl = drive - strafe + turn;
            double br = drive + strafe - turn;

            // normalize so no value exceeds 1.0
            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // ================== INTAKE BUTTON ==================
            if (gamepad1.right_bumper) {
                intake.setPower(1.0);   // intake in
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1.0);  // spit out
            } else {
                intake.setPower(0.0);   // stop when not pressed
            }

            // ================== SHOOTER TUNER ==================
            // --- shooter stepping with dpad ---
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;

            if (dpadUp && !prevDpadUp) {
                targetTPS += stepTPS;
            }
            if (dpadDown && !prevDpadDown) {
                targetTPS -= stepTPS;
            }
            prevDpadUp = dpadUp;
            prevDpadDown = dpadDown;

            targetTPS = Range.clip(targetTPS, 0, maxTPS);

            shooter.setVelocity(targetTPS);       // command (will spin reverse due to setDirection) [web:14]
            double actualTPS = shooter.getVelocity(); // measure

            // Convert to RPM so it is easier to remember/use elsewhere (assume 28 tpr, change if needed)
            double TICKS_PER_REV = 28.0;
            double targetRPM = targetTPS * 60.0 / TICKS_PER_REV;
            double actualRPM = actualTPS * 60.0 / TICKS_PER_REV;

            // --- turret target stepping with Y/A ---
            boolean y = gamepad1.y;
            boolean a = gamepad1.a;

            if (y && !prevY) {
                turretTargetTicks += turretStepTicks;
            }
            if (a && !prevA) {
                turretTargetTicks -= turretStepTicks;
            }
            prevY = y;
            prevA = a;

            int turretPos = turret.getCurrentPosition(); // ticks
            int turretError = turretTargetTicks - turretPos;

            // --- "lock in" values for copying into another opmode ---
            boolean x = gamepad1.x;
            if (x && !prevX) {
                telemetry.addLine("==== COPY THESE VALUES TO YOUR CODE ====");
                telemetry.addData("Shooter targetTPS", "%.1f", targetTPS);
                telemetry.addData("Shooter targetRPM", "%.1f", targetRPM);
                telemetry.addData("TurretTargetTicks", turretTargetTicks);
                telemetry.addLine("========================================");
            }
            prevX = x;

            // live telemetry
            telemetry.addData("Shooter targetTPS", "%.0f", targetTPS);
            telemetry.addData("Shooter actualTPS", "%.0f", actualTPS);
            telemetry.addData("Shooter targetRPM", "%.0f", targetRPM);
            telemetry.addData("Shooter actualRPM", "%.0f", actualRPM);
            telemetry.addData("Turret pos (ticks)", turretPos);
            telemetry.addData("Turret target (ticks)", turretTargetTicks);
            telemetry.addData("Turret error (ticks)", turretError);

            // extra telemetry
            telemetry.addData("Drive fl/fr/bl/br", "%.2f %.2f %.2f %.2f", fl, fr, bl, br);

            telemetry.update();
        }
    }
}