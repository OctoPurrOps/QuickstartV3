package org.firstinspires.ftc.teamcode.codes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@TeleOp(name = "🔵 blue-TeleOP-1z (Pedro GoalLock)")
public class bluetest extends OpMode {

    // Pedro follower drive
    private Follower follower;

    // Goal point (Pedro field coords)
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;

    // Starting pose
    private static final Pose START_POSE = new Pose(56, 8, Math.toRadians(90));

    // Heading convention fix
    private static final double HEADING_OFFSET = Math.toRadians(-90);

    // Turn controller (tune)
    private static final double K_TURN = 2.0;
    private static final double MAX_TURN = 1.0;

    // Hardware
    private Limelight3A limelight;

    private DcMotor intakeMotor;
    private DcMotor intakeMotor2;

    private DcMotorEx shooter1;
    private DcMotor aimMotor;

    private Servo sorterServo;
    private Servo shooterServo;
    private Servo pushservo;

    // ---- Push servo ----
    private final double PUSHSERVO_ACTIVE_POS = 70.0 / 360.0;
    private final double PUSHSERVO_INACTIVE_POS = 0.0;

    // Limelight turret constants
    private static final double DEADZONE = 2.0;
    private static final double MAX_TURRET_POWER = 0.35;
    private static final double FILTER_ALPHA = 0.85;
    private static final double KP = 0.025;
    private double filteredTx = 0;

    // Limelight distance parameters
    private static final double CAMERA_HEIGHT = 0.25;   // meters
    private static final double TARGET_HEIGHT = 0.90;   // meters
    private static final double CAMERA_ANGLE  = 20.0;   // degrees

    // Servo mapping (Limelight auto)
    private static final double SERVO_MIN = 0.1;
    private static final double SERVO_MAX = 0.40;
    private static final double DIST_MIN  = 0.4;
    private static final double DIST_MAX  = 3.0;

    // Shooter RPM range (Limelight auto values)
    private static final double RPM_MIN = 200;
    private static final double RPM_MAX = 1800;

    private boolean shooterOn = false;
    private boolean lastCross = false;

    // ------------------ DPAD controls you asked for ------------------
    // DPAD UP: aiming (HOLD) -> robot turns to face goal while held.
    // DPAD DOWN: toggle shooter 7000 TPS ON/OFF.
    // DPAD RIGHT: toggle shooter 2500 TPS ON/OFF.
    // DcMotorEx.setVelocity(angularRate) is ticks per second. [web:20]
    private static final double SHOOT_TPS_7000 = 7000;
    private static final double SHOOT_TPS_2500 = 2500;

    // Servo.setPosition() expects [0.0, 1.0]. [web:30]
    private static final double SHOOTER_SERVO_SHOOT_POS = 0.40;

    private boolean aimHold = false;

    private boolean shoot7000Enabled = false;
    private boolean shoot2500Enabled = false;

    private boolean dpadDownLatch = false;
    private boolean dpadRightLatch = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.update();

        // Intake
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);

        // Shooter motor PIDF
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidf = new PIDFCoefficients(31, 0, 0, 23);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        sorterServo = hardwareMap.get(Servo.class, "sortservo");
        pushservo = hardwareMap.get(Servo.class, "pushservo");
        pushservo.setPosition(PUSHSERVO_INACTIVE_POS);

        // Aim motor
        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setPower(0);

        // Shooter angle servo
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterServo.setPosition(SERVO_MIN);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        telemetry.addLine("✅ BLUE TELEOP (Pedro) READY");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        aimMotor.setPower(0);

        Pose p = follower.getPose();
        telemetry.addData("Init Pose", "x=%.1f y=%.1f h=%.1fdeg",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        // ------------------ DPAD UP: aiming hold ------------------
        aimHold = gamepad1.dpad_up;

        // ------------------ DPAD DOWN: toggle 7000 TPS ------------------
        if (gamepad1.dpad_down && !dpadDownLatch) {
            dpadDownLatch = true;
            shoot7000Enabled = !shoot7000Enabled;
            if (shoot7000Enabled) shoot2500Enabled = false; // make modes exclusive
        }
        if (!gamepad1.dpad_down) dpadDownLatch = false;

        // ------------------ DPAD RIGHT: toggle 2500 TPS ------------------
        if (gamepad1.dpad_right && !dpadRightLatch) {
            dpadRightLatch = true;
            shoot2500Enabled = !shoot2500Enabled;
            if (shoot2500Enabled) shoot7000Enabled = false; // make modes exclusive
        }
        if (!gamepad1.dpad_right) dpadRightLatch = false;

        boolean dpadShooting = shoot7000Enabled || shoot2500Enabled;

        // Driver translation
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;

        // Default turn
        double turn = -gamepad1.right_stick_x;

        // Aim-turn to face goal while holding DPAD UP
        if (aimHold) {
            Pose p = follower.getPose();
            double dx = GOAL_X - p.getX();
            double dy = GOAL_Y - p.getY();

            double targetHeading = Math.atan2(dy, dx) + HEADING_OFFSET;
            double error = AngleUnit.normalizeRadians(targetHeading - p.getHeading());
            turn = clamp(K_TURN * error, -MAX_TURN, MAX_TURN);
        }

        follower.setTeleOpDrive(forward, strafe, turn, true);

        // ------------------ Push servo (hold RB = out, release = in) ------------------
        if (gamepad1.right_bumper) {
            pushservo.setPosition(PUSHSERVO_ACTIVE_POS);
        } else {
            pushservo.setPosition(PUSHSERVO_INACTIVE_POS);
        }

        // ------------------ Intake (triggers + circle) ------------------
        double rt = gamepad1.right_trigger;
        double lt = gamepad1.left_trigger;

        double forwardPower = -0.8;
        double reversePower = 0.8;

        if (rt > 0.1) {
            intakeMotor.setPower(forwardPower * rt);
            intakeMotor2.setPower(0);
        } else if (lt > 0.1) {
            intakeMotor.setPower(forwardPower * lt);
            intakeMotor2.setPower(forwardPower * lt);
        } else if (gamepad1.circle) {
            intakeMotor.setPower(reversePower);
            intakeMotor2.setPower(reversePower);
        } else {
            intakeMotor.setPower(0);
            intakeMotor2.setPower(0);
        }

        // ------------------ Turret + shooter (Limelight) ------------------
        double turretPower = 0;
        double calculatedDistance = 0;
        double targetRPM = RPM_MIN;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();

            filteredTx = FILTER_ALPHA * filteredTx + (1 - FILTER_ALPHA) * tx;

            if (Math.abs(filteredTx) > DEADZONE) turretPower = filteredTx * KP;
            else turretPower = 0;

            double angleRad = Math.toRadians(CAMERA_ANGLE + ty);
            calculatedDistance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRad);

            double servoPos = SERVO_MIN
                    + (calculatedDistance - DIST_MIN) / (DIST_MAX - DIST_MIN) * (SERVO_MAX - SERVO_MIN);
            servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));

            // Only let Limelight move shooterServo when NOT in dpad shooting modes
            if (!dpadShooting) {
                shooterServo.setPosition(servoPos);
            }

            targetRPM = RPM_MIN
                    + (calculatedDistance - DIST_MIN) / (DIST_MAX - DIST_MIN) * (RPM_MAX - RPM_MIN);
            targetRPM = Math.max(RPM_MIN, Math.min(RPM_MAX, targetRPM));

            // Your old cross-toggle shooter (only used when not in dpad shooting modes)
            if (!dpadShooting) {
                if (gamepad1.cross && !lastCross) shooterOn = !shooterOn;
                lastCross = gamepad1.cross;
            } else {
                lastCross = gamepad1.cross; // still track edge so it doesn't “jump” after exiting
            }

        } else {
            turretPower = 0;
        }

        turretPower = clamp(turretPower, -MAX_TURRET_POWER, MAX_TURRET_POWER);
        aimMotor.setPower(turretPower);

        // ------------------ Shooter commands ------------------
        if (dpadShooting) {
            shooterServo.setPosition(SHOOTER_SERVO_SHOOT_POS); // [0..1] [web:30]
            shooter1.setVelocity(shoot7000Enabled ? SHOOT_TPS_7000 : SHOOT_TPS_2500); // ticks/sec [web:20]
        } else {
            shooter1.setVelocity(shooterOn ? targetRPM : 0);
        }

        // Telemetry
        Pose p = follower.getPose();
        telemetry.addData("AimHold(dpad_up)", aimHold);
        telemetry.addData("Shoot7000(dpad_down)", shoot7000Enabled);
        telemetry.addData("Shoot2500(dpad_right)", shoot2500Enabled);
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1fdeg",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("tx(f)", "%.2f", filteredTx);
        telemetry.addData("TurretPwr", "%.2f", turretPower);
        telemetry.addData("Dist(m)", "%.2f", calculatedDistance);
        telemetry.addData("Servo", "%.3f", shooterServo.getPosition());
        telemetry.addData("TargetRPM", (int) targetRPM);
        telemetry.addData("Shooter",
                dpadShooting ? (shoot7000Enabled ? "7000 TPS" : "2500 TPS")
                        : (shooterOn ? "AUTO ON" : "OFF"));
        telemetry.update();
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}