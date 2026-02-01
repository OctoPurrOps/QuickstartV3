package org.firstinspires.ftc.teamcode.codes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@TeleOp(name = "Goal Lock TeleOp (Pedro)")
public class track extends OpMode {

    private Follower follower;

    // Goal point (Pedro field coords are [0,144] and (0,0) is bottom-left) [web:46]
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;

    // Starting pose you requested
    private static final Pose START_POSE = new Pose(56, 8, Math.toRadians(90)); // heading in radians [web:83]

    // Toggle
    private boolean goalLock = false;
    private boolean dpadUpLatch = false;

    // Heading convention fix (Pedro: 0 rad faces +X/right; +90 faces +Y/up) [web:83]
    // Your robot/drive mapping appears to be rotated, so add +90deg.
    private static final double HEADING_OFFSET = Math.toRadians(-90);

    // Turn controller (tune)
    private static final double K_TURN = 2.0;
    private static final double MAX_TURN = 1.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap); // Pedro example pattern [web:61]
        follower.setStartingPose(START_POSE);
        follower.startTeleopDrive(); // Pedro example pattern [web:61]
    }

    @Override
    public void loop() {
        follower.update(); // call once per loop [web:61]

        // Toggle goal lock with dpad_up (edge)
        if (gamepad1.dpad_up && !dpadUpLatch) {
            dpadUpLatch = true;
            goalLock = !goalLock;
        }
        if (!gamepad1.dpad_up) dpadUpLatch = false;

        // Driver translation (left stick)
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;

        // Default driver turn (right stick)
        double turn = -gamepad1.right_stick_x;

        // Auto-turn to face the goal point
        if (goalLock) {
            Pose p = follower.getPose();

            double dx = GOAL_X - p.getX();
            double dy = GOAL_Y - p.getY();

            double targetHeading = Math.atan2(dy, dx) + HEADING_OFFSET;
            double error = AngleUnit.normalizeRadians(targetHeading - p.getHeading()); // [-pi, pi) [web:105]

            turn = clamp(K_TURN * error, -MAX_TURN, MAX_TURN);
        }

        // Robot-centric drive (last param true per Pedro example) [web:61]
        follower.setTeleOpDrive(forward, strafe, turn, true);

        telemetry.addData("GoalLock", goalLock);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading(deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}