package org.firstinspires.ftc.teamcode.codes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@Autonomous(name = "Blue Auto: GoalTrack -> Shoot -> Intake (NO PATHS)")
public class autoreel1 extends OpMode {

    private Follower follower;

    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;

    private static final Pose START_POSE = new Pose(56, 8, Math.toRadians(90));

    private static final double HEADING_OFFSET = Math.toRadians(-90);
    private static final double K_TURN = 2.0;
    private static final double MAX_TURN = 1.0;

    private int state = 0;
    private long stateStartMs = 0;

    private static final double AIM_TOL_RAD = Math.toRadians(2.0);
    private static final long AIM_HOLD_MS = 250;
    private static final long SPINUP_MS   = 700;
    private static final long FEED_MS     = 350;

    public static double targetTPS = 0;
    public static double stepTPS = 200;
    public static double maxTPS = 6000;

    private DcMotor intakeMotor;
    private DcMotor intakeMotor2;
    private DcMotorEx shooter1;
    private Servo shooterServo;

    private boolean started = false;   // gate logic until start is pressed

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setVelocity(0);

        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterServo.setPosition(0.35);

        targetTPS = 0;
        setState(0);
    }

    @Override
    public void start() {
        started = true;
        setState(0);
    }

    @Override
    public void loop() {
        // Keep follower updating each loop in a normal OpMode usage pattern. [page:0]
        follower.update();

        // Don’t run the autonomous actions before start; just hold still in init-loop.
        if (!started) {
            follower.setTeleOpDrive(0, 0, 0, true);
            shooter1.setVelocity(0);
            intakeOn(false);

            telemetry.addLine("Waiting for START...");
            telemetry.update();
            return;
        }

        // FULL FIX: guard against null Pose before calling getHeading/getX/getY. [web:44]
        Pose p = follower.getPose();
        if (p == null) {
            follower.setTeleOpDrive(0, 0, 0, true);
            shooter1.setVelocity(0);
            intakeOn(false);

            telemetry.addLine("ERROR: follower pose is null (localizer not ready / misconfigured).");
            telemetry.update();
            return;
        }

        double dx = GOAL_X - p.getX();
        double dy = GOAL_Y - p.getY();
        double targetHeading = Math.atan2(dy, dx) + HEADING_OFFSET;
        double err = AngleUnit.normalizeRadians(targetHeading - p.getHeading());
        double turnCmd = clamp(K_TURN * err, -MAX_TURN, MAX_TURN);

        switch (state) {
            case 0: // aim
                follower.setTeleOpDrive(0, 0, turnCmd, true);
                shooter1.setVelocity(0);
                intakeOn(false);

                if (Math.abs(err) < AIM_TOL_RAD) {
                    if (millis() - stateStartMs > AIM_HOLD_MS) setState(1);
                } else {
                    stateStartMs = millis();
                }
                break;

            case 1: // spin up
                follower.setTeleOpDrive(0, 0, turnCmd, true);
                rampShooterToMax();
                intakeOn(false);

                if (millis() - stateStartMs > SPINUP_MS) setState(2);
                break;

            case 2: // feed
                follower.setTeleOpDrive(0, 0, turnCmd, true);
                rampShooterToMax();
                intakeOn(true);

                if (millis() - stateStartMs > FEED_MS) {
                    intakeOn(false);
                    setState(-1);
                }
                break;

            default: // done
                follower.setTeleOpDrive(0, 0, 0, true);
                shooter1.setVelocity(0);
                intakeOn(false);
                break;
        }

        telemetry.addData("state", state);
        telemetry.addData("poseX", p.getX());
        telemetry.addData("poseY", p.getY());
        telemetry.addData("poseHeadingDeg", Math.toDegrees(p.getHeading()));
        telemetry.addData("targetTPS", targetTPS);
        telemetry.addData("shooterTPS", shooter1.getVelocity());
        telemetry.update();
    }

    private void rampShooterToMax() {
        targetTPS += stepTPS;
        if (targetTPS > maxTPS) targetTPS = maxTPS;
        shooter1.setVelocity(targetTPS);
    }

    private void setState(int s) {
        state = s;
        stateStartMs = millis();
    }

    private long millis() {
        return System.currentTimeMillis();
    }

    private void intakeOn(boolean on) {
        double p = on ? -0.8 : 0.0;
        intakeMotor.setPower(p);
        intakeMotor2.setPower(p);
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}