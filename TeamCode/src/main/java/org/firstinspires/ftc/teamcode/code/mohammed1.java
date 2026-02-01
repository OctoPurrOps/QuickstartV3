package org.firstinspires.ftc.teamcode.code;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled

@TeleOp(name = "Turret Goal Lock (Pedro)")
public class mohammed1 extends OpMode {

    // --- Pedro ---
    private Follower follower;

    // --- Turret ---
    private DcMotor aimMotor;

    // Goal point (Pedro field coords [0..144]) [web:46]
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;

    // Start pose
    private static final Pose START_POSE = new Pose(56, 8, Math.toRadians(90)); // radians [web:83]

    // Toggle
    private boolean turretLock = false;
    private boolean dpadUpLatch = false;

    // Heading convention fix you found works: -90 deg (keep) [web:83]
    private static final double HEADING_OFFSET = Math.toRadians(-90);

    // --- Turret calibration (TUNE THESE) ---
    // ticks per rad = ticks per rev / (2*pi) * gearRatio [web:116]
    private static final double AIM_TICKS_PER_RAD = 85.6; // TODO tune for your turret

    private static final int AIM_MIN_TICKS = -2000; // TODO set safe soft limits
    private static final int AIM_MAX_TICKS =  2000;

    private static final double AIM_POWER = 0.6;

    // For telemetry
    private double desiredTurretRad = 0.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap); // Pedro example pattern [web:61]
        follower.setStartingPose(START_POSE);

        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Encoder position control (RUN_TO_POSITION) [web:116][web:137]
        aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aimMotor.setTargetPosition(0);
        aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        aimMotor.setPower(AIM_POWER);

        follower.startTeleopDrive(); // Pedro TeleOp pattern [web:61]
    }

    @Override
    public void loop() {
        follower.update(); // once per loop [web:61]

        // Toggle turret lock with dpad_up (edge)
        if (gamepad1.dpad_up && !dpadUpLatch) {
            dpadUpLatch = true;
            turretLock = !turretLock;
        }
        if (!gamepad1.dpad_up) dpadUpLatch = false;

        // Drive (normal)
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;
        follower.setTeleOpDrive(forward, strafe, turn, true); // robot-centric param like example [web:61]

        // Turret aim at goal
        desiredTurretRad = 0.0;
        if (turretLock) {
            Pose p = follower.getPose();

            // Field heading from robot -> goal
            double dx = GOAL_X - p.getX();
            double dy = GOAL_Y - p.getY();
            double goalFieldHeading = Math.atan2(dy, dx) + HEADING_OFFSET;

            // turret angle needed relative to robot heading
            desiredTurretRad = AngleUnit.normalizeRadians(goalFieldHeading - p.getHeading()); // wrap [-pi,pi) [web:105]

            int targetTicks = (int) Math.round(desiredTurretRad * AIM_TICKS_PER_RAD);
            targetTicks = clampInt(targetTicks, AIM_MIN_TICKS, AIM_MAX_TICKS);

            aimMotor.setTargetPosition(targetTicks);
            // keep RUN_TO_POSITION; motor holds target [web:137]
        }

        // ---- Telemetry ----
        telemetry.addData("TurretLock", turretLock);
        telemetry.addData("AIM_TICKS_PER_RAD (set)", "%.3f", AIM_TICKS_PER_RAD);

        int cur = aimMotor.getCurrentPosition();
        int tgt = aimMotor.getTargetPosition();
        telemetry.addData("AimTicks cur/tgt", "%d / %d", cur, tgt); // encoder ticks [web:137]

        if (turretLock) {
            telemetry.addData("DesiredTurretRad", "%.3f", desiredTurretRad);
            telemetry.addData("DesiredTurretDeg", "%.1f", Math.toDegrees(desiredTurretRad));
            if (Math.abs(desiredTurretRad) > 1e-3) {
                telemetry.addData("TicksPerRad (instant)", "%.3f", tgt / desiredTurretRad);
            }
        }

        telemetry.addData("Pose", follower.getPose());
        telemetry.update(); // send to DS [web:130]
    }

    private static int clampInt(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}