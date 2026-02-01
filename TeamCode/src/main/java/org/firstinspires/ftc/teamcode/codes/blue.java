package org.firstinspires.ftc.teamcode.codes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "🔵 blue-TeleOP-1z ")
public class blue extends LinearOpMode {

    // Mecanum motors
    private Limelight3A limelight;
    private DcMotor intakeMotor;
    private DcMotor intakeMotor2;

    private DcMotorEx shooter1;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor aimMotor;
    private IMU imu;
    private Servo sorterServo;
    private Servo shooterServo;
    private Servo pushservo;

    // ---- Intake positions ----

    private boolean xPressed = false;

    private boolean intakeForward = true;
    private boolean lastCircle = false;


    // ---- Launch positions ----


    // ---- Push servo ----
    private final double PUSHSERVO_ACTIVE_POS = 70.0 / 360.0;
    private final double PUSHSERVO_INACTIVE_POS = 0.0;

    // ---- Intake ----
    private final double INTAKE_POWER = 1.0;

    // Button latches
    private boolean circleLatch = false;      // used for LAUNCH cycling (now left stick button)
    private boolean dpadRightLatch = false;   // used for intake advance (now left bumper)

    // Pushing state
    private boolean isPushing = false;

    // Turret constants
    private static final double DEADZONE = 2.0;
    private static final double MAX_TURRET_POWER = 0.35;
    private static final double FILTER_ALPHA = 0.85;
    private static final double KP = 0.025;

    private double filteredTx = 0;

    // Limelight distance parameters
    private static final double CAMERA_HEIGHT = 0.25;   // meters
    private static final double TARGET_HEIGHT = 0.90;   // meters
    private static final double CAMERA_ANGLE  = 20.0;   // degrees

    // Servo mapping
    private static final double SERVO_MIN = 0.1;   // minimum angle
    private static final double SERVO_MAX = 0.40;  // maximum angle
    private static final double DIST_MIN  = 0.4;   // meters (close)
    private static final double DIST_MAX  = 3.0;   // meters (far)

    // Shooter RPM range (PIDF-controlled) - UPDATED TO 500-1800
    private static final double RPM_MIN = 200;   // Close shots (very soft)
    private static final double RPM_MAX = 1800;  // Far shots

    boolean shooterOn = false;    // Shooter running state
    boolean lastbuttom = false;   // Last frame state of the button




    @Override
    public void runOpMode() {

        // --- Drive Motors ---
//        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
//        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2");
//        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);



        // ═══════════════════════════════════════════════════════════════
        // PIDF SETUP FOR SHOOTER MOTOR
        // ═══════════════════════════════════════════════════════════════
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        // Apply tuned PIDF coefficients
        PIDFCoefficients pidf = new PIDFCoefficients(31, 0, 0, 23);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        // ═══════════════════════════════════════════════════════════════

        sorterServo = hardwareMap.get(Servo.class, "sortservo");
        pushservo = hardwareMap.get(Servo.class, "pushservo");

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        pushservo.setPosition(PUSHSERVO_INACTIVE_POS);

        // --- Turret Motor ---
        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Shooter Servo ---
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterServo.setPosition(SERVO_MIN); // default start

        // --- Limelight ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        telemetry.addLine("✅ 🔴 RED TELEOP READY");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("🎯 PIDF: P=31, I=0, D=0, F=23");
        telemetry.addLine("🚀 RPM Range: " + (int)RPM_MIN + "-" + (int)RPM_MAX);
        telemetry.addLine("📐 Servo: " + SERVO_MIN + "-" + SERVO_MAX);
        telemetry.addLine("═══════════════════════════════");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ------------------ Mecanum Drive ------------------
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x * 0.8;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // --------------------------------------------------
            // 🔵 INTAKE AUTO USING COLOR SENSOR
            // --------------------------------------------------


            // NOTE: You currently have left_stick_button used elsewhere too.
            // This line is kept as-is from your code (it will now share the button with launch cycling).

            if (gamepad1.right_bumper && !xPressed) {
                xPressed = true;
                pushservo.setPosition(PUSHSERVO_ACTIVE_POS);
                servoDelay(250);
                pushservo.setPosition(PUSHSERVO_INACTIVE_POS);
            } else if (!gamepad1.right_bumper) {
                xPressed = false;
            }



            // --------------------------------------------------
            // LEFT BUMPER → Intake advance (SWAPPED from left stick button)
            // --------------------------------------------------
            if (gamepad1.left_bumper && !dpadRightLatch && !isPushing) {
                dpadRightLatch = true;
                isPushing = true;

                // Advance intake after push

                isPushing = false;
            }
            if (!gamepad1.left_bumper) {
                dpadRightLatch = false;
            }

            double rt = gamepad1.right_trigger;
            double lt = gamepad1.left_trigger;

            double forwardPower = -0.8;  // Intake speed
            double reversePower = 0.8; // Outtake speed

            if (rt > 0.1) {
                // Right trigger: intakeMotor only, forward
                intakeMotor.setPower(forwardPower * rt);
                intakeMotor2.setPower(0);
            } else if (lt > 0.1) {
                // Left trigger: both forward
                intakeMotor.setPower(forwardPower * lt);
                intakeMotor2.setPower(forwardPower * lt);
            } else if (gamepad1.circle) {
                // Circle: both reverse (outtake), always
                intakeMotor.setPower(reversePower);
                intakeMotor2.setPower(reversePower);
            } else {
                intakeMotor.setPower(0);
                intakeMotor2.setPower(0);
            }



            // ------------------ Turret Control (Limelight) ------------------
            double turretPower = 0;
            double calculatedDistance = 0;
            double targetRPM = RPM_MIN;

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();

                // Low-pass filter for smoothness
                filteredTx = FILTER_ALPHA * filteredTx + (1 - FILTER_ALPHA) * tx;

                if (Math.abs(filteredTx) > DEADZONE) {
                    turretPower = filteredTx * KP;
                }

                // --- Calculate distance ---
                double angleRad = Math.toRadians(CAMERA_ANGLE + ty);
                calculatedDistance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRad);

                // --- Map distance to servo angle ---
                double servoPos = SERVO_MIN + (calculatedDistance - DIST_MIN) / (DIST_MAX - DIST_MIN)
                        * (SERVO_MAX - SERVO_MIN);
                servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));
                shooterServo.setPosition(servoPos);

                // --- Calculate RPM based on distance ---
                targetRPM = RPM_MIN + (calculatedDistance - DIST_MIN) / (DIST_MAX - DIST_MIN)
                        * (RPM_MAX - RPM_MIN);
                targetRPM = Math.max(RPM_MIN, Math.min(RPM_MAX, targetRPM));

                // --- Toggle shooter ON/OFF ---
                if (gamepad1.cross && !lastbuttom) {
                    shooterOn = !shooterOn; // Toggle state
                }
                lastbuttom = gamepad1.cross;

                // --- Apply velocity with PIDF control ---
                // IMPORTANT: setVelocity() is in ticks/sec unless using AngleUnit overload. [web:19]
                if (shooterOn) {
                    shooter1.setVelocity(targetRPM);
                } else {
                    shooter1.setVelocity(0);
                }
            }

            // Clamp turret power
            turretPower = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, turretPower));
            aimMotor.setPower(turretPower);

            // ------------------ Enhanced Telemetry ------------------
            telemetry.addData("🔴 Alliance", "RED");
            telemetry.addData("━━━━━━━━━━━━━━━━━━", "");
            telemetry.addData("tx", String.format("%.2f", filteredTx));
            telemetry.addData("Turret Power", String.format("%.2f", turretPower));
            telemetry.addData("━━━━━━━━━━━━━━━━━━", "");
            telemetry.addData("📐 Servo Pos", String.format("%.3f", shooterServo.getPosition()));
            telemetry.addData("📏 Distance (m)", String.format("%.2f", calculatedDistance));
            telemetry.addData("━━━━━━━━━━━━━━━━━━", "");
            telemetry.addData("🎯 Target RPM", (int) targetRPM);
            telemetry.addData("⚙ Actual RPM", (int) shooter1.getVelocity());
            telemetry.addData("🚀 Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("━━━━━━━━━━━━━━━━━━", "");
            telemetry.update();
        }
    }



    // ---- Safe busy-wait delay (short) ----
    private void servoDelay(long ms) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < ms) {
            // small busy wait; keep short to avoid watchdog issues
        }
    }
}