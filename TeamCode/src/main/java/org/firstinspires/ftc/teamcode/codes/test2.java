package org.firstinspires.ftc.teamcode.codes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "🔴 RED-TeleOP-FIXED-v3-DEBUG ")
public class test2 extends LinearOpMode {

    // Hardware
    private Limelight3A limelight;
    private DcMotor intakeMotor, intakeMotor2;
    private DcMotorEx shooter1;
    private DcMotor frontLeft, frontRight, backLeft, backRight, aimMotor;
    private Servo sorterServo, shooterServo, pushservo;

    // States
    private boolean xPressed = false;
    private boolean dpadRightLatch = false;
    private boolean isPushing = false;
    private boolean shooterOn = false;
    private boolean lastCross = false;
    private double advanceStartTime = -1.0;
    private double filteredTx = 0;

    // Constants
    private final double PUSHSERVO_ACTIVE_POS = 70.0 / 360.0;
    private final double PUSHSERVO_INACTIVE_POS = 0.0;
    private final double INTAKE_POWER = 1.0;
    private final double INTAKE_ADVANCE_POWER = 0.8;
    private final double INTAKE_ADVANCE_TIME = 0.3;

    private static final double DEADZONE = 2.0;
    private static final double MAX_TURRET_POWER = 0.35;
    private static final double FILTER_ALPHA = 0.85;
    private static final double KP = 0.025;

    // DECODE Calibration - MEASURE THESE!
    private static final double CAMERA_HEIGHT = 0.45;
    private static final double TARGET_HEIGHT = 1.05;
    private static final double CAMERA_ANGLE = 15.0;

    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 0.3;
    private static final double DIST_MIN = 0.4;
    private static final double DIST_MAX = 3.5;
    private static final double RPM_MIN = 200;
    private static final double RPM_MAX = 1800;
    private static final double TICKS_PER_REV = 2786.0;

    private ElapsedTime elapsedTime = new ElapsedTime();
    private double calculatedDistance = 2.0;
    private double targetRPM = RPM_MIN;

    @Override
    public void runOpMode() {
        initHardware();

        // COLOR pipeline for reliable ty!
        limelight.pipelineSwitch(3);
        limelight.start();

        telemetry.addLine("✅ 🔴 RED v3 - DEBUG MODE");
        telemetry.addLine("📏 Calib: Hc=" + CAMERA_HEIGHT + " Ht=" + TARGET_HEIGHT + " A=" + CAMERA_ANGLE + "°");
        telemetry.addLine("🎯 Pipeline 0 - CONFIGURE HSV FOR RED GOALS!");
        telemetry.addLine("X = toggle shooter (independent)");
        telemetry.update();

        waitForStart();
        elapsedTime.reset();

        while (opModeIsActive()) {
            double currentTime = elapsedTime.seconds();
            driveMecanum();
            handlePushServo();
            handleIntakeAdvance(currentTime);
            handleIntakeTriggers();
            handleLimelightShooter();
            updateTelemetry();
        }
    }

    private void initHardware() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidf = new PIDFCoefficients(31, 0, 0, 23);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        sorterServo = hardwareMap.get(Servo.class, "sortservo");
        pushservo = hardwareMap.get(Servo.class, "pushservo");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        pushservo.setPosition(PUSHSERVO_INACTIVE_POS);
        shooterServo.setPosition(SERVO_MIN);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    private void driveMecanum() {
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x * 0.8;
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeft.setPower((y + x + rx) / denom);
        backLeft.setPower((y - x + rx) / denom);
        frontRight.setPower((y - x - rx) / denom);
        backRight.setPower((y + x - rx) / denom);
    }

    private void handlePushServo() {
        if (gamepad1.right_bumper && !xPressed) {
            xPressed = true;
            pushservo.setPosition(PUSHSERVO_ACTIVE_POS);
            sleep(250);
            pushservo.setPosition(PUSHSERVO_INACTIVE_POS);
        } else if (!gamepad1.right_bumper) {
            xPressed = false;
        }
    }

    private void handleIntakeAdvance(double currentTime) {
        if (gamepad1.left_bumper && !dpadRightLatch && !isPushing) {
            dpadRightLatch = true;
            isPushing = true;
            advanceStartTime = currentTime;
            intakeMotor.setPower(INTAKE_ADVANCE_POWER);
            intakeMotor2.setPower(INTAKE_ADVANCE_POWER);
        } else if (!gamepad1.left_bumper) {
            dpadRightLatch = false;
        }
        if (isPushing && currentTime - advanceStartTime >= INTAKE_ADVANCE_TIME) {
            intakeMotor.setPower(0);
            intakeMotor2.setPower(0);
            isPushing = false;
        }
    }

    private void handleIntakeTriggers() {
        double rt = gamepad1.right_trigger;
        double lt = gamepad1.left_trigger;
        double fwd = -0.8;
        double rev = 0.8;
        if (rt > 0.1) {
            intakeMotor.setPower(fwd * rt);
            intakeMotor2.setPower(0);
        } else if (lt > 0.1) {
            intakeMotor.setPower(fwd * lt);
            intakeMotor2.setPower(fwd * lt);
        } else if (gamepad1.circle) {
            intakeMotor.setPower(rev);
            intakeMotor2.setPower(rev);
        } else {
            intakeMotor.setPower(0);
            intakeMotor2.setPower(0);
        }
    }

    // FIXED: Always processes tx/ty when available, independent toggle
    private void handleLimelightShooter() {
        double turretPower = 0;
        calculatedDistance = DIST_MAX / 2;
        targetRPM = RPM_MIN;

        LLResult result = limelight.getLatestResult();
        boolean hasResult = result != null;
        boolean isValid = hasResult && result.isValid();

        if (hasResult) {
            double tx = result.getTx();
            double ty = result.getTy();

            // ALWAYS track on tx
            filteredTx = FILTER_ALPHA * filteredTx + (1 - FILTER_ALPHA) * tx;
            if (Math.abs(filteredTx) > DEADZONE) {
                turretPower = filteredTx * KP;
            }

            // Distance calc when reasonable ty
            if (ty > -45 && ty < 45) {
                double angleRad = Math.toRadians(CAMERA_ANGLE + ty);
                if (angleRad > 0.01) {
                    calculatedDistance = Math.max(DIST_MIN, Math.min(DIST_MAX,
                            (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRad)));
                }
            }

            // Servo always follows distance
            double servoPos = SERVO_MIN + (calculatedDistance - DIST_MIN) / (DIST_MAX - DIST_MIN) * (SERVO_MAX - SERVO_MIN);
            shooterServo.setPosition(Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos)));

            targetRPM = RPM_MIN + (calculatedDistance - DIST_MIN) / (DIST_MAX - DIST_MIN) * (RPM_MAX - RPM_MIN);
        }

        // Shooter toggle INDEPENDENT of vision
        if (gamepad1.cross && !lastCross) {
            shooterOn = !shooterOn;
        }
        lastCross = gamepad1.cross;

        double targetVel = shooterOn ? targetRPM * TICKS_PER_REV / 60.0 : 0;
        shooter1.setVelocity(targetVel);

        turretPower = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, turretPower));
        aimMotor.setPower(turretPower);
    }

    private void updateTelemetry() {
        LLResult result = limelight.getLatestResult();
        boolean hasResult = result != null;
        telemetry.addData("Result", hasResult ? "OK" : "NULL");
        telemetry.addData("Valid", hasResult ? result.isValid() : false);
        telemetry.addData("Fiducials", hasResult ? result.getFiducialResults().size() : -1);
        telemetry.addData("Pipeline", limelight.pipelineSwitch(3));
        if (hasResult) {
            telemetry.addData("Raw tx/ty", String.format("%.1f/%.1f", result.getTx(), result.getTy()));
        }
        telemetry.addData("Filtered tx", String.format("%.1f", filteredTx));
        telemetry.addData("Turret Pwr", String.format("%.2f", aimMotor.getPower()));
        telemetry.addData("Dist/Servo", String.format("%.2fm / %.3f", calculatedDistance, shooterServo.getPosition()));
        telemetry.addData("RPM tgt/act", String.format("%.0f / %.0f", targetRPM, shooter1.getVelocity() * 60 / TICKS_PER_REV));
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("Intake", isPushing ? "ADVANCING" : "IDLE");
        telemetry.update();
    }
}
