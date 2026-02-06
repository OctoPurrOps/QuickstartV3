package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "BLUE-TELE")
public class BLUE_tele extends OpMode {

    GoBildaPinpointDriver pinpoint;


    // Shooter variables
    DcMotorEx shooter;
    DcMotorEx shooter2;
    double shooterSpeed = 0.0;
    DcMotor intake;

    // Angle servo
    Servo angleServo;
    Servo gate;
    double anglePos = 0;
    public double currentHeading, currentX, currentY, distance, deltaY, deltaX, velocityX, velocityY, relative_angle;


    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor aimMotor;

    private static final double GOAL_X = 0;
    private static final double GOAL_Y = 144.0;
    private static final double HEADING_OFFSET = Math.toRadians(0);

    private static final double TICKS_PER_REV = 1627;
    private static final double AIM_TICKS_PER_RAD = TICKS_PER_REV / (2.0 * Math.PI);
    private static final int AIM_MIN_TICKS = -2000;
    private static final int AIM_MAX_TICKS = 2000;
    private static final double AIM_POWER = 1;

    private static final double LOOKAHEAD_TIME = 0.0;
    double P=8, F=16.6;

    private boolean turretLock = false;
    private boolean dpadUpLatch = false;

    private boolean servoExtended = false;
    private boolean xPrevious = false;

    double y, x, rotate, mp, ms, flp, blp, frp, brp;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(5.14, -3.918, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aimMotor.setTargetPosition(0);
        aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        aimMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        aimMotor.setPower(AIM_POWER);

        intake = hardwareMap.get(DcMotor.class, "intake");


        shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        angleServo = hardwareMap.get(Servo.class, "shooterServo");
        gate = hardwareMap.get(Servo.class, "gate");



        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients=new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        angleServo.setPosition(anglePos);
        gate.setPosition(0);

    }

    boolean shooterIsOn=false;

    @Override
    public void loop() {
        update_blue();
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        flp = y + x + rotate;
        blp = y - x + rotate;
        frp = y - x - rotate;
        brp = y + x - rotate;

        mp = 1.0;
        ms = 1.0;
        mp = Math.max(mp, Math.abs(flp));
        mp = Math.max(mp, Math.abs(blp));
        mp = Math.max(mp, Math.abs(frp));
        mp = Math.max(mp, Math.abs(brp));
        frontLeft.setPower(ms * (flp / mp));
        backLeft.setPower(ms * (blp / mp));
        frontRight.setPower(ms * (frp / mp));
        backRight.setPower(ms * (brp / mp));
        if(shooterIsOn) {
            shooterSpeed=6.85*distance+720;
        }
        else {
            shooterSpeed=0;
        }

        // Shooter control: continuous at set speed, buttons adjust
        shooter.setVelocity(shooterSpeed);  // Convert to rad/s (adjust RPM factor for your flywheel) [web:23]
        shooter2.setVelocity(shooterSpeed);  // Convert to rad/s (adjust RPM factor for your flywheel) [web:23]

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

        if(gamepad1.crossWasPressed()) {
            shooterIsOn=!shooterIsOn;
        }

        boolean xCurrent = gamepad1.cross;

        if (xCurrent && !xPrevious) {  // Rising edge: button just pressed
            if (servoExtended) {
                gate.setPosition(0.0);
            } else {
                gate.setPosition(0.7);
            }
            servoExtended = !servoExtended;  // Toggle state
        }

        xPrevious = xCurrent;  // Update for next loop



        double rt = gamepad1.right_trigger;
        double lt = gamepad1.left_trigger;

        double forwardPower = -1;
        double reversePower = 1;

        if (rt > 0.1) {
            intake.setPower(forwardPower * rt);
//            intake2.setPower(0);
        } else if (lt > 0.1) {
            intake.setPower(forwardPower * lt);
//            intake2.setPower(-0.6 * lt);
        } else if (gamepad1.circle) {
            intake.setPower(reversePower);
//            intake2.setPower(reversePower);
        } else {
            intake.setPower(0);
//            intake2.setPower(0);
        }
        // Toggle
        if (gamepad1.dpad_up && !dpadUpLatch) {
            dpadUpLatch = true;
            turretLock = !turretLock;
        }
        if (!gamepad1.dpad_up) dpadUpLatch = false;

        // Drive

        if (turretLock) {
            Pose p = new Pose(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getHeading(AngleUnit.RADIANS));

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
        telemetry.addData("vel: ", shooter.getVelocity() + shooter2.getVelocity());
        telemetry.addData("target vel: ", shooterSpeed);
        telemetry.addData("pos x: ", currentX);
        telemetry.addData("pos y: ", currentY);
        telemetry.addData("heading: ", currentHeading);
        telemetry.addData("distance: ", distance);
        telemetry.addData("anglePos: ", anglePos);
        telemetry.update();

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