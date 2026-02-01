package org.firstinspires.ftc.teamcode.codes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@TeleOp(name="PP TeleOp: X AutoShot Full", group="TeleOp")
public class test extends OpMode {

    // ---------- Hardware names (EDIT to match your config) ----------
    private static final String FLYWHEEL_NAME = "shooter1";
    private static final String HOOD_SERVO_NAME = "shooterServo";
    // If you also have a turret motor, add it here:
    // private static final String TURRET_NAME = "turret";

    // ---------- Goal / field ----------
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;

    // ---------- Start pose ----------
    private static final double START_X = 56.0;
    private static final double START_Y = 8.0;
    private static final double START_HEADING_RAD = 0.0;

    // ---------- Shooter gearing / cap ----------
    private static final double MOTOR_FREE_RPM = 6000.0;
    private static final double OUTPUT_FREE_RPM = MOTOR_FREE_RPM * (18.0 / 24.0); // 4500
    private static final double RPM_MAX_CMD = OUTPUT_FREE_RPM;
    private static final double RPM_MIN_CMD = 0.0;

    // ---------- Hood servo limits ----------
    // You said: highest position is 0.50
    private static final double HOOD_MAX_POS = 0.50;
    private static final double HOOD_MIN_POS = 0.10; // tune safe minimum
    // Servo setPosition() expects [0..1]. [web:113]

    // ---------- Calibration tables (REPLACE with your tested data) ----------
    // Distance -> output RPM
    private static final double[] DIST_IN_RPM =  { 24,  48,  72,  96, 120, 144 };
    private static final double[] RPM_TABLE   =  {2500, 2900, 3300, 3700, 4100, 4500 };

    // Distance -> hood servo position (0..1)
    private static final double[] DIST_IN_HOOD = { 24,  48,  72,  96, 120, 144 };
    private static final double[] HOOD_TABLE   = {0.50, 0.47, 0.44, 0.40, 0.36, 0.33 };

    // ---------- Optional offsets (tune) ----------
    private static final double GOAL_AIM_OFFSET_DEG = 0.0;
    private static final double TURRET_MECH_ZERO_OFFSET_DEG = 0.0;

    // ---------- Pedro + hardware ----------
    private Follower follower;
    private DcMotorEx flywheel;
    private Servo hood;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING_RAD));
        follower.update();

        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hood = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);
        hood.setPosition(HOOD_MAX_POS);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();      // ---- Drive (Pedro TeleOp) ----
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;
        follower.setTeleOpDrive(forward, strafe, turn, true);

        // ---- Pose -> distance/aim ----
        Pose p = follower.getPose();
        double robotX = p.getX();
        double robotY = p.getY();
        double robotHeadingDeg = wrapDeg(Math.toDegrees(p.getHeading()));

        double dx = GOAL_X - robotX;
        double dy = GOAL_Y - robotY;

        double distance = Math.hypot(dx, dy);

        double targetFieldDeg = wrapDeg(Math.toDegrees(Math.atan2(dy, dx)));
        double turretCmdDeg = wrapDeg(targetFieldDeg - robotHeadingDeg
                + TURRET_MECH_ZERO_OFFSET_DEG
                + GOAL_AIM_OFFSET_DEG);

        // ---- Compute shooter setpoints from distance ----
        double rpmCmd  = lerpTable(distance, DIST_IN_RPM, RPM_TABLE);
        double hoodCmd = lerpTable(distance, DIST_IN_HOOD, HOOD_TABLE);

        rpmCmd  = clamp(rpmCmd, RPM_MIN_CMD, RPM_MAX_CMD);
        hoodCmd = clamp(hoodCmd, HOOD_MIN_POS, HOOD_MAX_POS);

        // ---- X button enables auto-shot ----
        if (gamepad1.x) { // standard FTC gamepad button read [web:102]
            // Hood position from distance
            hood.setPosition(hoodCmd);

            // Flywheel velocity from RPM
            // DcMotorEx supports setVelocity(value, AngleUnit). [web:53]
            flywheel.setVelocity(rpmToRadPerSec(rpmCmd), AngleUnit.RADIANS);
        } else {
            // Idle
            flywheel.setVelocity(0.0, AngleUnit.RADIANS);
            hood.setPosition(HOOD_MAX_POS);
        }

        telemetry.addData("X pressed", gamepad1.x);
        telemetry.addData("Pose X/Y", "%.1f / %.1f", robotX, robotY);
        telemetry.addData("Heading deg", "%.1f", robotHeadingDeg);
        telemetry.addData("Distance", "%.2f", distance);
        telemetry.addData("Turret cmd deg (calc)", "%.1f", turretCmdDeg);
        telemetry.addData("RPM cmd", "%.0f (cap %.0f)", rpmCmd, RPM_MAX_CMD);
        telemetry.addData("Hood cmd", "%.3f (max %.2f)", hoodCmd, HOOD_MAX_POS);

        // Actual motor speed readback (optional)
        telemetry.addData("Flywheel rad/s (meas)", "%.1f", flywheel.getVelocity(AngleUnit.RADIANS));

        telemetry.update();
    }

    // ---------- Math helpers ----------
    private static double rpmToRadPerSec(double rpm) {
        return rpm * 2.0 * Math.PI / 60.0;
    }

    private static double lerpTable(double x, double[] xs, double[] ys) {
        if (x <= xs[0]) return ys[0];
        int n = xs.length;
        if (x >= xs[n - 1]) return ys[n - 1];

        int i = 0;
        while (i < n - 1 && x > xs[i + 1]) i++;

        double x0 = xs[i], x1 = xs[i + 1];
        double y0 = ys[i], y1 = ys[i + 1];
        double t = (x - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    private static double wrapDeg(double deg) {
        deg %= 360.0;
        if (deg >= 180.0) deg -= 360.0;
        if (deg < -180.0) deg += 360.0;
        return deg;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}