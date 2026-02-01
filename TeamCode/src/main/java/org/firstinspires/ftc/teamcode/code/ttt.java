package org.firstinspires.ftc.teamcode.code;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled

@TeleOp(name = "Turret EDGE TEST (v999)")
public class ttt extends OpMode {

    GoBildaPinpointDriver pinpoint;

    // Shooter variables
    DcMotorEx shooter;
    double shooterSpeed = 0.0;  // Adjustable 0-1 (or velocity RPM/60)
    DcMotor intake;
    DcMotor intake2;

    // Angle servo
    Servo angleServo;
    double anglePos = 0;  // 0.0 to 1.0 range
    public double currentHeading, currentX, currentY, distance, deltaY, deltaX, velocityX, velocityY, relative_angle;


    private Follower follower;
    private DcMotor aimMotor;

    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;

    // START POSE
    private static final Pose START_POSE = new Pose(27, 130, Math.toRadians(144));
    private static final double HEADING_OFFSET = Math.toRadians(0);

    private static final double TICKS_PER_REV = 1627;
    private static final double AIM_TICKS_PER_RAD = TICKS_PER_REV / (2.0 * Math.PI);
    private static final int AIM_MIN_TICKS = -2000;
    private static final int AIM_MAX_TICKS = 2000;
    private static final double AIM_POWER = 1;

    // DISABLED PREDICTION FOR TESTING (Set to 0.0)
    // If this fixes the wall issue, the problem was Velocity Prediction overshooting the wall.
    private static final double LOOKAHEAD_TIME = 0.0;

    private boolean turretLock = false;
    private boolean dpadUpLatch = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aimMotor.setTargetPosition(0);
        aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        aimMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        aimMotor.setPower(AIM_POWER);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake2.setDirection(DcMotor.Direction.REVERSE);

        follower.startTeleopDrive();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        angleServo = hardwareMap.get(Servo.class, "shooterServo");



        pinpoint.setOffsets(5.14, -3.918, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 27, 130, AngleUnit.DEGREES, 144));
        // Init shooter
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Velocity PID [web:26]
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Init servo
        angleServo.setPosition(anglePos);

    }

    @Override
    public void loop() {

        update_blue();
        shooterSpeed=6.85*distance+1050;

        // Shooter control: continuous at set speed, buttons adjust
        shooter.setVelocity(shooterSpeed);  // Convert to rad/s (adjust RPM factor for your flywheel) [web:23]
        if (gamepad1.dpadLeftWasPressed()) shooterSpeed+=10;
        if (gamepad1.dpadRightWasPressed()) shooterSpeed-=10;

        // Servo angle adjust (incremental)
        if (distance < 55) {
            anglePos = 0.00;
        } else if (distance < 65) {
            anglePos = 0.01;
        } else if (distance < 85) {
            anglePos = 0.02;
        } else if (distance < 110) {
            anglePos = 0.03;
        } else if (distance < 125) {
            anglePos = 0.04;
        } else {
            anglePos = 0.06;
        }


        double rt = gamepad1.right_trigger;
        double lt = gamepad1.left_trigger;

        double forwardPower = -0.8;
        double reversePower = 0.8;

        if (rt > 0.1) {
            intake.setPower(forwardPower * rt);
            intake2.setPower(0);
        } else if (lt > 0.1) {
            intake.setPower(forwardPower * lt);
            intake2.setPower(forwardPower * lt);
        } else if (gamepad1.circle) {
            intake.setPower(reversePower);
            intake2.setPower(reversePower);
        } else {
            intake.setPower(0);
            intake2.setPower(0);
        }
        // Toggle
        if (gamepad1.dpad_up && !dpadUpLatch) {
            dpadUpLatch = true;
            turretLock = !turretLock;
        }
        if (!gamepad1.dpad_up) dpadUpLatch = false;

        // Drive
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        if (turretLock) {
            Pose p = follower.getPose();

            // DIRECT AIM (No Prediction) to test coordinate logic
            double dx = GOAL_X - p.getX();
            double dy = GOAL_Y - p.getY();

            // IMPORTANT: Diagnostic Telemetry
            // If dy goes negative, you drove "past" the goal Y=144.
            telemetry.addData("Goal Vector DX", "%.1f", dx);
            telemetry.addData("Goal Vector DY", "%.1f", dy);

            double goalFieldHeading = Math.atan2(dy, dx) + HEADING_OFFSET;
            double desiredTurretRad = AngleUnit.normalizeRadians(goalFieldHeading - p.getHeading());

            int targetTicks = (int) Math.round(desiredTurretRad * AIM_TICKS_PER_RAD);
            targetTicks = clampInt(targetTicks, AIM_MIN_TICKS, AIM_MAX_TICKS);

            aimMotor.setTargetPosition(targetTicks);
        }

        telemetry.addData("TurretLock", turretLock);
        telemetry.addData("Pose", follower.getPose());
        angleServo.setPosition(anglePos);
        telemetry.addData("vel: ", shooter.getVelocity());
        telemetry.addData("target vel: ", shooterSpeed);
        telemetry.addData("pos x: ", currentX);
        telemetry.addData("pos y: ", currentY);
        telemetry.addData("heading: ", currentHeading);
        telemetry.addData("distance: ", distance);
        telemetry.addData("anglePos: ", anglePos);
        telemetry.update();
        follower.update();

    }

    private static int clampInt(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public void update_blue() {
        pinpoint.update();
        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        currentX = pinpoint.getPosX(DistanceUnit.INCH);
        currentY = pinpoint.getPosY(DistanceUnit.INCH);
        velocityX = pinpoint.getVelX(DistanceUnit.INCH);
        velocityY = pinpoint.getVelY(DistanceUnit.INCH);
        deltaY = 144 - currentY;
        deltaX = 0 - currentX;
        relative_angle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
}