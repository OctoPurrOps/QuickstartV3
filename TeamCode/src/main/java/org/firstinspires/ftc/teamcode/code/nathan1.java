package org.firstinspires.ftc.teamcode.code;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled

@TeleOp(name = "Turret AutoAim FINAL DeadWheels", group = "TeleOp")
public class nathan1 extends OpMode {

    // ===== GOAL POSITION (FIELD COORDINATES) =====
    public static final double GOAL_X = 0; // 72 + 8
    public static final double GOAL_Y = 144; // 72 + 35

    // ===== TURRET CONFIG =====
    // Encoder 117 ticks × 5:1 gear ratio
    public static final double TICKS_PER_TURRET_REV = 1627;//538.0;
    public static final double TURRET_ZERO_OFFSET_RAD = 0.0;
    public static final double TURRET_DIRECTION = -1.0; // عدلها لو لف غلط

    // ===== PID
    public static final double kP = 0.04;
    public static final double kD = 0.0001;
    public static final double MAX_POWER = 1;
    public static final double MIN_POWER = 0.1;
    public static final double DEADBAND_RAD = Math.toRadians(0.5);

    private Follower follower;
    private DcMotor turretMotor;

    private double lastError = 0.0;
    private double lastTime = 0.0;

    @Override
    public void init() {

        // ===== DEAD WHEELS FOLLOWER (NO IMU) =====
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));
        follower.update();

        // ===== TURRET MOTOR =====
        turretMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lastTime = getRuntime();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // ===== DRIVE =====
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        // ===== ROBOT POSE (DEAD WHEELS) =====
        Pose robot = follower.getPose();
        double robotHeading = robot.getHeading();

        // ===== FIELD ANGLE TO GOAL =====
        double dx = GOAL_X - robot.getX();
        double dy = GOAL_Y - robot.getY();
        double fieldAngle = Math.atan2(dy, dx);

        // ===== TURRET TARGET (ROBOT RELATIVE) =====
        double turretTarget = wrapRad(
                robotHeading - fieldAngle + TURRET_ZERO_OFFSET_RAD
        );

        // ===== CURRENT TURRET ANGLE =====
        double turretAngle = wrapRad(ticksToRad(turretMotor.getCurrentPosition()));
        double error = wrapRad(turretTarget - turretAngle);

        // ===== TIME STEP =====
        double now = getRuntime();
        double dt = Math.max(now - lastTime, 0.001);

        // ===== PD CONTROL =====
        double derivative = (error - lastError) / dt;
        double power;


        power = kP * error + kD * derivative;

        power = Range.clip(power, -MAX_POWER, MAX_POWER);
        turretMotor.setPower(TURRET_DIRECTION * power);

        lastError = error;
        lastTime = now;

        // ===== TELEMETRY =====
        telemetry.addLine("=== DEAD WHEELS AUTO AIM ===");
        telemetry.addData("Robot X", "%.1f", robot.getX());
        telemetry.addData("Robot Y", "%.1f", robot.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotHeading));
        telemetry.addData("Turret Target", "%.1f°", Math.toDegrees(turretTarget));
        telemetry.addData("Turret Angle", "%.1f°", Math.toDegrees(turretAngle));
        telemetry.addData("Error", "%.2f°", Math.toDegrees(error));
        telemetry.addData("Power", "%.3f", power);
        telemetry.update();
    }

    // ===== HELPERS =====
    private double ticksToRad(int ticks) {
        return (ticks / TICKS_PER_TURRET_REV) * 2.0 * Math.PI;
    }

    private double wrapRad(double angle) {
        angle %= (2.0 * Math.PI);
        if (angle > Math.PI) angle -= 2.0 * Math.PI;
        if (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}