package org.firstinspires.ftc.teamcode.codes;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@TeleOp(name = "Turret Goal Lock (Pedro) - Fast PD")
public class track2 extends OpMode {

    private Follower follower;
    private DcMotorEx aimMotor;

    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;

    private static final Pose START_POSE = new Pose(56, 8, Math.toRadians(90));
    private static final double HEADING_OFFSET = Math.toRadians(-90);

    private static final double TICKS_PER_REV = 538.0;
    private static final double TICKS_PER_RAD = TICKS_PER_REV / (2.0 * Math.PI);

    private static final int AIM_MIN_TICKS = -2000;
    private static final int AIM_MAX_TICKS = 2000;

    // --- PD + feedforward (tune these) ---
    private static final double kP = 0.6;      // power per rad
    private static final double kD = 0.01;     // power per (rad/s)
    private static final double kS = 0.2;     // static friction feedforward (power)
    private static final double MAX_POWER = 0.85;

    private boolean turretLock = false;
    private boolean dpadUpLatch = false;

    private double desiredTurretRad = 0.0;
    private double lastErr = 0.0;
    private long lastTimeNs = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.startTeleopDrive(); // call once like Pedro example [web:7]

        aimMotor = hardwareMap.get(DcMotorEx.class, "aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aimMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // use your own loop [web:4]

        lastTimeNs = System.nanoTime();
    }

    @Override
    public void loop() {
        follower.update(); // once per loop like Pedro example [web:7]

        if (gamepad1.dpad_up && !dpadUpLatch) {
            dpadUpLatch = true;
            turretLock = !turretLock;

            // avoid derivative kick on enable
            lastErr = 0.0;
            lastTimeNs = System.nanoTime();
        }
        if (!gamepad1.dpad_up) dpadUpLatch = false;

        // Drive (unchanged)
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;
        follower.setTeleOpDrive(forward, strafe, turn, true);

        // --- Turret tracking ---
        double motorPower = 0.0;
        if (turretLock) {
            Pose p = follower.getPose();
            double dx = GOAL_X - p.getX();
            double dy = GOAL_Y - p.getY();

            double goalFieldHeading = Math.atan2(dy, dx) + HEADING_OFFSET;
            desiredTurretRad = AngleUnit.normalizeRadians(goalFieldHeading - p.getHeading());

            // Current turret angle from encoder
            int ticks = aimMotor.getCurrentPosition();
            double turretRad = ticks / TICKS_PER_RAD;

            // Error, wrapped to [-pi, pi]
            double err = AngleUnit.normalizeRadians(desiredTurretRad - turretRad);

            // Soft limits: prevent commanding further into the stop
            if ((ticks <= AIM_MIN_TICKS && err < 0) || (ticks >= AIM_MAX_TICKS && err > 0)) {
                err = 0.0;
            }

            long now = System.nanoTime();
            double dt = (now - lastTimeNs) / 1e9;
            if (dt < 1e-4) dt = 1e-4;

            double errRate = (err - lastErr) / dt;

            // PD + static feedforward
            motorPower = kP * err - kD * errRate;

            // add kS only when we actually want to move
            if (Math.abs(err) > Math.toRadians(0.5)) {
                motorPower += Math.signum(motorPower) * kS;
            }

            motorPower = clamp(motorPower, -MAX_POWER, MAX_POWER);

            lastErr = err;
            lastTimeNs = now;
        }

        aimMotor.setPower(motorPower);

        telemetry.addData("TurretLock", turretLock);
        telemetry.addData("MotorPower", "%.2f", motorPower);
        telemetry.addData("Turret ticks", aimMotor.getCurrentPosition());
        telemetry.addData("Desired (deg)", "%.1f", Math.toDegrees(desiredTurretRad));
        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}