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

@TeleOp(name = "Turret AutoAim 180-Limit", group = "TeleOp")
public class md1 extends OpMode {

    public static final double GOAL_X = 0;
    public static final double GOAL_Y = 144;

    public static final double TICKS_PER_TURRET_REV = 1627;
    public static final double TURRET_DIRECTION = -1.0;

    // ===== LIMITS (180 Degrees total range) =====
    // نضع حدود عند 90 و -90 درجة
    public static final double MAX_LIMIT_RAD = Math.toRadians(90);
    public static final double MIN_LIMIT_RAD = Math.toRadians(-90);

    public static final double kP = 4;
    public static final double kD = 1;
    public static final double MAX_POWER = 1;
    public static final double DEADBAND_RAD = Math.toRadians(0.5);

    private Follower follower;
    private DcMotor turretMotor;
    private double lastError = 0.0;
    private double lastTime = 0.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));
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
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        Pose robot = follower.getPose();
        double robotHeading = robot.getHeading();

        double dx = GOAL_X - robot.getX();
        double dy = GOAL_Y - robot.getY();
        double fieldAngle = Math.atan2(dy, dx);

        // حساب الهدف المطلوب
        double turretTarget = wrapRad(fieldAngle - robotHeading);

        // قراءة زاوية التيريت الحالية (بدون wrap لأننا نحتاج القيمة الحقيقية للمقارنة مع الحدود)
        double currentAngleRad = ticksToRad(turretMotor.getCurrentPosition());

        // ===== تطبيق حدود الحركة (Soft Limits) =====
        // نقوم بحصر الهدف داخل النطاق المسموح به (-90 إلى 90)
        double constrainedTarget = Range.clip(turretTarget, MIN_LIMIT_RAD, MAX_LIMIT_RAD);

        // حساب الخطأ بناءً على الهدف المحصور
        double error = wrapRad(constrainedTarget - currentAngleRad);

        double now = getRuntime();
        double dt = Math.max(now - lastTime, 0.001);

        double derivative = (error - lastError) / dt;
        double power = kP * error + kD * derivative;

        // حماية إضافية: إذا وصلنا للحد الأقصى، نمنع القوة في ذلك الاتجاه
        if (currentAngleRad >= MAX_LIMIT_RAD && power > 0) power = 0;
        if (currentAngleRad <= MIN_LIMIT_RAD && power < 0) power = 0;

        power = Range.clip(power, -MAX_POWER, MAX_POWER);
        turretMotor.setPower(TURRET_DIRECTION * power);

        lastError = error;
        lastTime = now;

        telemetry.addData("Turret Angle Deg", Math.toDegrees(currentAngleRad));
        telemetry.addData("Target Constrained", Math.toDegrees(constrainedTarget));
        telemetry.addData("Status", (currentAngleRad >= MAX_LIMIT_RAD || currentAngleRad <= MIN_LIMIT_RAD) ? "LIMIT REACHED" : "TRACKING");
        telemetry.update();
    }

    private double ticksToRad(int ticks) {
        return (ticks / TICKS_PER_TURRET_REV) * 2.0 * Math.PI;
    }

    private double wrapRad(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }
}