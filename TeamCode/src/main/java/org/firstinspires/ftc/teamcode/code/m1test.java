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

@TeleOp(name = "m1 Turret AutoAim Pinpoint FINAL", group = "TeleOp")
public class m1test extends OpMode {

    // DECODE red goal approx. (Pedro tiles; verify field print)
    public static final double GOAL_X = 0;  // 72 + 8
    public static final double GOAL_Y = 144; // 144 - 16

    // TURRET
    public static final double TICKS_PER_TURRET_REV = 1627;
    public static final double TURRET_ZERO_OFFSET_RAD = 0.0;
    public static final double TURRET_DIRECTION = -1.0;

    // PD
    public static final double kP = 0.04;
    public static final double kD = 0.0001;
    public static final double MAX_POWER = 1.0;
    public static final double DEADBAND_RAD = Math.toRadians(0.5);

    private Follower follower;
    private DcMotor turretMotor;
    private double lastError = 0.0;
    private double lastTime = 0.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(72, 72, 0));  // Field center start
        follower.update();

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
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        Pose robot = follower.getPose();
        double robotHeading = robot.getHeading();

        double dx = GOAL_X - robot.getX();
        double dy = GOAL_Y - robot.getY();
        double fieldAngle = Math.atan2(dy, dx);

        double turretTarget = wrapRad(robotHeading - fieldAngle + TURRET_ZERO_OFFSET_RAD);
        double turretAngle = wrapRad(ticksToRad(turretMotor.getCurrentPosition()));
        double error = wrapRad(turretTarget - turretAngle);

        double power = 0;
        if (Math.abs(error) > DEADBAND_RAD) {
            double now = getRuntime();
            double dt = Math.max(now - lastTime, 0.001);
            double derivative = (error - lastError) / dt;
            power = Range.clip(kP * error + kD * derivative, -MAX_POWER, MAX_POWER);
            turretMotor.setPower(TURRET_DIRECTION * power);
            lastError = error;
            lastTime = now;
        } else {
            turretMotor.setPower(0);
        }

        telemetry.addLine("PINPOINT AUTO AIM");
        telemetry.addData("X", "%.1f", robot.getX());
        telemetry.addData("Y", "%.1f", robot.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotHeading));
        telemetry.addData("Target", "%.1f°", Math.toDegrees(turretTarget));
        telemetry.addData("Angle", "%.1f°", Math.toDegrees(turretAngle));
        telemetry.addData("Error", "%.2f°", Math.toDegrees(error));
        telemetry.addData("Power", "%.3f", power);
        telemetry.update();
    }

    private double ticksToRad(int ticks) {
        return (ticks / TICKS_PER_TURRET_REV) * 2 * Math.PI;
    }

    private double wrapRad(double angle) {
        angle %= 2 * Math.PI;
        if (angle > Math.PI) angle -= 2 * Math.PI;
        if (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
