package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled

@TeleOp
public class BLUE2 extends OpMode {
    GoBildaPinpointDriver pinpoint;

    // Shooter variables
    DcMotorEx shooter;
    double shooterSpeed = 0.0;  // Adjustable 0-1 (or velocity RPM/60)



    // Angle servo
    Servo angleServo;
    double anglePos = 0;  // 0.0 to 1.0 range
    public double currentHeading, currentX, currentY, distance, deltaY, deltaX, velocityX, velocityY, relative_angle;
    @Override
    public void init() {
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
        angleServo.setPosition(anglePos);
        telemetry.addData("vel: ", shooter.getVelocity());
        telemetry.addData("target vel: ", shooterSpeed);
        telemetry.addData("pos x: ", currentX);
        telemetry.addData("pos y: ", currentY);
        telemetry.addData("heading: ", currentHeading);
        telemetry.addData("distance: ", distance);
        telemetry.addData("anglePos: ", anglePos);
        telemetry.update();
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