//package org.firstinspires.ftc.teamcode;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
//
//@TeleOp(name="PP TeleOp: Precise AutoAim", group="TeleOp")
//public class PPTeleOpPreciseAutoAim extends OpMode {
//
//    // ---- Goal ----
//    private static final double GOAL_X = 0.0;
//    private static final double GOAL_Y = 144.0;
//
//    // ---- Start ----
//    private static final double START_X = 56.0, START_Y = 8.0;
//    private static final double START_HEADING_RAD = 0.0;
//
//    // ---- Pedro ----
//    private Follower follower;
//
//    // ---- Shooter hood servo ----
//    private Servo hoodServo;
//    private static final String HOOD_SERVO_NAME = "hood";
//
//    // Servo limits (tune so you never hit hard-stops; servos can be damaged if forced). [web:40]
//    private static final double HOOD_MIN = 0.10;
//    private static final double HOOD_MAX = 0.90;
//
//    // ---- Turret offsets ----
//    private static final double TURRET_MECH_ZERO_OFFSET_DEG = 0.0; // turret "0" relative to robot forward
//    private static final double GOAL_AIM_OFFSET_DEG = 0.0;         // trim
//
//    // ---- Distance->RPM (example points; replace with tested values) ----
//    private static final double[] DIST_IN_RPM = { 24,  48,  72,  96, 120, 144 };
//    private static final double[] RPM_TABLE   = {2500, 2900, 3300, 3700, 4100, 4500 };
//
//    // ---- Distance->Hood (servo position) (example points; replace with tested values) ----
//    // Servo position is usually 0..1. Use calibration points + interpolate. [web:35]
//    private static final double[] DIST_IN_HOOD = { 24,  48,  72,  96, 120, 144 };
//    private static final double[] HOOD_TABLE   = {0.78,0.72,0.66,0.60,0.54,0.50 };
//
//    // ---- Optional shoot-on-the-move lead compensation ----
//    // If you can estimate robot field velocity, you can “lead” by shifting the aim point. [web:2]
//    private static final boolean ENABLE_LEAD = false;
//    private static final double LEAD_TIME_SEC = 0.18; // tune (time-of-flight + mechanism latency)
//
//    // Very simple velocity estimate from pose delta (works ok if loop timing stable)
//    private Pose lastPose = null;
//    private long lastTimeNanos = 0;
//
//    @Override
//    public void init() {
//        Constants.setConstants(FConstants.class, LConstants.class); [web:16]
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING_RAD)); [web:16]
//
//        hoodServo = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);
//
//        follower.update(); [web:16]
//        lastPose = follower.getPose(); [web:16]
//        lastTimeNanos = System.nanoTime();
//    }
//
//    @Override
//    public void start() {
//        follower.startTeleopDrive(); [web:16]
//        lastPose = follower.getPose(); [web:16]
//        lastTimeNanos = System.nanoTime();
//    }
//
//    @Override
//    public void loop() {
//        follower.update(); [web:16]
//        Pose p = follower.getPose(); [web:16]
//
//        // ---- Drive (same idea as Pedro TeleOp example) ----
//        double forward = -gamepad1.left_stick_y;
//        double strafe  = -gamepad1.left_stick_x;
//        double turn    = -gamepad1.right_stick_x;
//        follower.setTeleOpMovementVectors(forward, strafe, turn, true);
//
//        // ---- Estimate field velocity from pose delta (optional) ----
//        long now = System.nanoTime();
//        double dt = (now - lastTimeNanos) / 1e9;
//        if (dt < 1e-6) dt = 1e-6;
//
//        double vx = 0, vy = 0;
//        if (lastPose != null) {
//            vx = (p.getX() - lastPose.getX()) / dt;
//            vy = (p.getY() - lastPose.getY()) / dt;
//        }
//        lastPose = p;
//        lastTimeNanos = now;
//
//        // ---- Target point (optionally lead) ----
//        double targetX = GOAL_X;
//        double targetY = GOAL_Y;
//
//        if (ENABLE_LEAD) {
//            // Lead by aiming where the robot will be after LEAD_TIME_SEC (simple approximation). [web:2]
//            double futureX = p.getX() + vx * LEAD_TIME_SEC;
//            double futureY = p.getY() + vy * LEAD_TIME_SEC;
//
//            // Equivalent to aiming from future pose; so use dx/dy from future pose to goal:
//            // (implemented by shifting target the opposite way)
//            targetX = GOAL_X; // goal fixed
//            targetY = GOAL_Y;
//            // We'll compute dx/dy using future pose below.
//            aimAndSet(p, futureX, futureY, targetX, targetY);
//        } else {
//            aimAndSet(p, p.getX(), p.getY(), targetX, targetY);
//        }
//    }
//
//    private void aimAndSet(Pose robotPose, double useRobotX, double useRobotY, double goalX, double goalY) {
//        double dx = goalX - useRobotX;
//        double dy = goalY - useRobotY;
//
//        double distance = Math.hypot(dx, dy);
//
//        // atan2 gives correct quadrant angle from dx/dy. [web:37]
//        double targetFieldDeg = wrapDeg(Math.toDegrees(Math.atan2(dy, dx))); [web:37]
//        double robotHeadingDeg = wrapDeg(Math.toDegrees(robotPose.getHeading()));
//
//        double turretCmdDeg = wrapDeg(targetFieldDeg - robotHeadingDeg
//                + TURRET_MECH_ZERO_OFFSET_DEG
//                + GOAL_AIM_OFFSET_DEG);
//
//        double rpmCmd = lerpTable(distance, DIST_IN_RPM, RPM_TABLE);
//        double hoodCmd = lerpTable(distance, DIST_IN_HOOD, HOOD_TABLE);
//
//        // Clamp hood to safe range; helps avoid hitting hard stops. [web:40]
//        hoodCmd = clamp(hoodCmd, HOOD_MIN, HOOD_MAX); [web:40]
//        hoodServo.setPosition(hoodCmd);
//
//        // TODO: Replace with your turret PID + flywheel velocity controller
//        // setTurretAngleDeg(turretCmdDeg);
//        // setFlywheelRpm(rpmCmd);
//
//        telemetry.addData("Pose X/Y", "%.1f , %.1f", robotPose.getX(), robotPose.getY());
//        telemetry.addData("Heading deg", "%.1f", robotHeadingDeg);
//        telemetry.addData("Distance", "%.2f", distance);
//        telemetry.addData("Target field deg", "%.1f", targetFieldDeg);
//        telemetry.addData("Turret cmd deg", "%.1f", turretCmdDeg);
//        telemetry.addData("RPM cmd", "%.0f", rpmCmd);
//        telemetry.addData("Hood servo", "%.3f", hoodCmd);
//        telemetry.update();
//    }
//
//    // Generic linear interpolation table (recommended approach for hood+rpm tuning). [web:35]
//    private static double lerpTable(double x, double[] xs, double[] ys) {
//        if (x <= xs[0]) return ys[0];
//        int n = xs.length;
//        if (x >= xs[n - 1]) return ys[n - 1];
//
//        int i = 0;
//        while (i < n - 1 && x > xs[i + 1]) i++;
//
//        double x0 = xs[i], x1 = xs[i + 1];
//        double y0 = ys[i], y1 = ys[i + 1];
//        double t = (x - x0) / (x1 - x0);
//        return y0 + t * (y1 - y0);
//    }
//
//    private static double wrapDeg(double deg) {
//        deg %= 360.0;
//        if (deg >= 180.0) deg -= 360.0;
//        if (deg < -180.0) deg += 360.0;
//        return deg;
//    }
//
//    private static double clamp(double v, double lo, double hi) {
//        return Math.max(lo, Math.min(hi, v));
//    }
//}