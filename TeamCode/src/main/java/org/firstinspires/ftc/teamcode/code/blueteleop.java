package org.firstinspires.ftc.teamcode.code;

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

@TeleOp(name = "bluetele")
public class blueteleop extends OpMode {

    private Follower follower;

    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;

    private static final Pose START_POSE = new Pose(56, 8, Math.toRadians(90));

    private static final double HEADING_OFFSET = Math.toRadians(-90);

    private static final double K_TURN = 0.5;
    private static final double MAX_TURN = 0.7;

    private static final double DRIVE_MULT = 0.85;
    private static final double STRAFE_MULT = 0.70;
    private static final double STICK_DEADZONE = 0.06;

    private Limelight3A limelight;

    private DcMotor intakeMotor;
    private DcMotor intakeMotor2;

    private DcMotorEx shooter1;
    private DcMotor aimMotor;

    private Servo sorterServo;
    private Servo shooterServo;
    private Servo pushservo;

    private final double PUSHSERVO_ACTIVE_POS = 70.0 / 360.0;
    private final double PUSHSERVO_INACTIVE_POS = 0.0;

    private static final double DEADZONE = 2.0;
    private static final double MAX_TURRET_POWER = 0.35;
    private static final double FILTER_ALPHA = 0.85;
    private static final double KP = 0.025;
    private double filteredTx = 0;

    private static final double CAMERA_HEIGHT = 0.25;
    private static final double TARGET_HEIGHT = 0.90;
    private static final double CAMERA_ANGLE  = 20.0;

    private static final double SERVO_MIN = 0.1;
    private static final double SERVO_MAX = 0.40;
    private static final double DIST_MIN  = 0.4;
    private static final double DIST_MAX  = 3.0;

    private static final double RPM_MIN = 200;
    private static final double RPM_MAX = 1800;

    private boolean shooterOn = false;
    private boolean lastCross = false;

    private static final double SHOOT_TPS_7000 = 7000;
    private static final double SHOOT_TPS_2500 = 2500;
    private static final double SHOOTER_SERVO_DPAD_SHOOT_POS = 0.40;

    private boolean shoot7000Enabled = false;
    private boolean shoot2500Enabled = false;
    private boolean dpadDownLatch = false;
    private boolean dpadRightLatch = false;

    private static final double M1_TPS = 2500;
    private static final double M1_SERVO_ON = 0.30;
    private static final double M1_SERVO_OFF = 0.0;

    private boolean m1Mode = false;
    private boolean m1Latch = false;

    // NEW: dpad‑up lock-at-zero state (same behavior as your red tele)
    private boolean dpadUpLatch = false;
    private boolean lockAimAtZero = false;
    private static final double AIM_HOLD_POWER = 0.5;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.update();

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
        pushservo.setPosition(PUSHSERVO_INACTIVE_POS);

        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMPORTANT: start in encoder mode for Limelight control
        // (we only switch to RUN_TO_POSITION when lockAimAtZero is enabled)
        aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aimMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aimMotor.setPower(0);

        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterServo.setPosition(SERVO_MIN);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        telemetry.addLine("READY");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        aimMotor.setPower(0);

        Pose p = follower.getPose();
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1fdeg",
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

        // DPAD UP: toggle "lock aim at 0"
        if (gamepad1.dpad_up && !dpadUpLatch) {
            dpadUpLatch = true;
            lockAimAtZero = !lockAimAtZero;

            if (lockAimAtZero) {
                // Define current physical angle as 0 and HOLD it
                aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                aimMotor.setTargetPosition(0);
                aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                aimMotor.setPower(AIM_HOLD_POWER);
            } else {
                // Release hold, go back to Limelight power control
                aimMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                aimMotor.setPower(0);
            }
        }
        if (!gamepad1.dpad_up) dpadUpLatch = false;

        if (gamepad1.square && !m1Latch) {
            m1Latch = true;
            m1Mode = !m1Mode;
            if (!m1Mode) shooterServo.setPosition(M1_SERVO_OFF);
        }
        if (!gamepad1.square) m1Latch = false;

        if (gamepad1.dpad_down && !dpadDownLatch) {
            dpadDownLatch = true;
            shoot7000Enabled = !shoot7000Enabled;
            if (shoot7000Enabled) shoot2500Enabled = false;
        }
        if (!gamepad1.dpad_down) dpadDownLatch = false;

        if (gamepad1.dpad_right && !dpadRightLatch) {
            dpadRightLatch = true;
            shoot2500Enabled = !shoot2500Enabled;
            if (shoot2500Enabled) shoot7000Enabled = false;
        }
        if (!gamepad1.dpad_right) dpadRightLatch = false;

        boolean dpadShooting = shoot7000Enabled || shoot2500Enabled;

        double rawForward = -gamepad1.left_stick_y;
        double rawStrafe  = -gamepad1.left_stick_x;
        double rawTurn    = -gamepad1.right_stick_x;

        double forward = dz(rawForward, STICK_DEADZONE) * DRIVE_MULT;
        double strafe  = dz(rawStrafe,  STICK_DEADZONE) * DRIVE_MULT * STRAFE_MULT;
        double turn    = dz(rawTurn,    STICK_DEADZONE) * DRIVE_MULT;

        forward = clip(forward, -1, 1);
        strafe  = clip(strafe,  -1, 1);
        turn    = clip(turn,    -1, 1);

        if (m1Mode) {
            Pose p = follower.getPose();
            double dx = GOAL_X - p.getX();
            double dy = GOAL_Y - p.getY();
            double targetHeading = Math.atan2(dy, dx) + HEADING_OFFSET;
            double error = AngleUnit.normalizeRadians(targetHeading - p.getHeading());
            turn = clamp(K_TURN * error, -MAX_TURN, MAX_TURN);
        }

        follower.setTeleOpDrive(forward, strafe, turn, true);

        if (gamepad1.right_bumper) pushservo.setPosition(PUSHSERVO_ACTIVE_POS);
        else pushservo.setPosition(PUSHSERVO_INACTIVE_POS);

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

            double servoPos = SERVO_MIN + (calculatedDistance - DIST_MIN)
                    / (DIST_MAX - DIST_MIN) * (SERVO_MAX - SERVO_MIN);
            servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));

            boolean servoOverridden = m1Mode || dpadShooting;
            if (!servoOverridden) shooterServo.setPosition(servoPos);

            targetRPM = RPM_MIN + (calculatedDistance - DIST_MIN)
                    / (DIST_MAX - DIST_MIN) * (RPM_MAX - RPM_MIN);
            targetRPM = Math.max(RPM_MIN, Math.min(RPM_MAX, targetRPM));

            if (!m1Mode && !dpadShooting) {
                if (gamepad1.cross && !lastCross) shooterOn = !shooterOn;
                lastCross = gamepad1.cross;
            } else {
                lastCross = gamepad1.cross;
            }
        } else {
            turretPower = 0;
        }

        // Only apply Limelight turret power when NOT locked at 0
        if (!lockAimAtZero) {
            turretPower = clamp(turretPower, -MAX_TURRET_POWER, MAX_TURRET_POWER);
            if (aimMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                aimMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            aimMotor.setPower(turretPower);
        }

        if (m1Mode) {
            shooterServo.setPosition(M1_SERVO_ON);
            shooter1.setVelocity(M1_TPS);
        } else if (dpadShooting) {
            shooterServo.setPosition(SHOOTER_SERVO_DPAD_SHOOT_POS);
            shooter1.setVelocity(shoot7000Enabled ? SHOOT_TPS_7000 : SHOOT_TPS_2500);
        } else {
            shooter1.setVelocity(shooterOn ? targetRPM : 0);
        }

        Pose p = follower.getPose();
        telemetry.addData("M1(square)", m1Mode);
        telemetry.addData("7000", shoot7000Enabled);
        telemetry.addData("2500", shoot2500Enabled);
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1fdeg",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("tx", "%.2f", filteredTx);
        telemetry.addData("Turret", "%.2f", turretPower);
        telemetry.addData("Dist", "%.2f", calculatedDistance);
        telemetry.addData("Servo", "%.3f", shooterServo.getPosition());
        telemetry.addData("AutoRPM", (int) targetRPM);
        telemetry.addData("Aim Pos", aimMotor.getCurrentPosition());
        telemetry.addData("LockAimAtZero", lockAimAtZero);
        telemetry.update();
    }

    private double dz(double v, double dz) {
        if (Math.abs(v) <= dz) return 0.0;
        return Math.copySign((Math.abs(v) - dz) / (1.0 - dz), v);
    }

    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}