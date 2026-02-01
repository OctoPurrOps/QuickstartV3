package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled

@Autonomous(name = "Auto: Turret Shooter FINAL", group = "Competition")
public class blueau extends OpMode {

    private Follower follower;
    private Paths paths;

    private DcMotor aimMotor;
    private DcMotorEx shooter;
    private Servo angleServo;
    private DcMotor intake;
    private DcMotor intake2;

    private int pathState = 0;
    private long stateStartTime = 0;

    // Turret & Shooter Constants
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;
    private static final double TURRET_HEADING_OFFSET = Math.toRadians(-90);
    private static final double TICKS_PER_REV = 1627;
    private static final double AIM_TICKS_PER_RAD = TICKS_PER_REV / (2.0 * Math.PI);
    private static final int AIM_MIN_TICKS = -2000;
    private static final int AIM_MAX_TICKS = 2000;
    private static final double AIM_POWER = 1.0;
    private static final double INTAKE_POWER = -0.8;

    private double shooterSpeed = 0.0;
    private double anglePos = 0.0;
    private double distanceToGoal = 0.0;

    // Start position
    private static final Pose START_POSE = new Pose(27.000, 130.000, Math.toRadians(144));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START_POSE);
        paths = new Paths(follower);
        initHardware();
        telemetry.addLine("✅ Turret Shooter Auto Ready!");
        telemetry.update();
    }

    @Override
    public void start() {
        // Start shooter immediately
        updateShooter(START_POSE);
        follower.followPath(paths.Path1, true);
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        manageIntakeTiming();

        Pose currentPose = follower.getPose();
        updateTurret(currentPose);
        updateShooter(currentPose);

        telemetry.addData("Path State", pathState);
        telemetry.addData("Distance", "%.1f", distanceToGoal);
        telemetry.addData("Shooter Vel", "%.0f", shooter.getVelocity());
        telemetry.addData("Aim Pos", aimMotor.getCurrentPosition());
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        if (!follower.isBusy()) {
            switch (pathState) {
                case 0: // Path 1 done -> Start Path 2 with intake
                    follower.followPath(paths.Path2, true);
                    pathState = 1;
                    stateStartTime = System.currentTimeMillis();
                    intake.setPower(INTAKE_POWER);
                    intake2.setPower(INTAKE_POWER);
                    break;

                case 1: // Path 2 done -> Stop intakes, start Path 3
                    follower.followPath(paths.Path3, true);
                    pathState = 2;
                    intake.setPower(0);
                    intake2.setPower(0);
                    break;

                case 2:
                    follower.followPath(paths.Path4, true);
                    pathState = 3;
                    break;

                case 3:
                    follower.followPath(paths.Path5, true);
                    pathState = 4;
                    break;

                case 4:
                    follower.followPath(paths.Path6, true);
                    pathState = 5;
                    break;

                case 5:
                    follower.followPath(paths.Path7, true);
                    pathState = 6;
                    break;

                case 6:
                    follower.followPath(paths.Path8, true);
                    pathState = 7;
                    break;

                case 7:
                    follower.followPath(paths.Path9, true);
                    pathState = 8;
                    break;

                case 8:
                    follower.followPath(paths.Path10, true);
                    pathState = 9;
                    break;

                case 9:
                    follower.followPath(paths.Path11, true);
                    pathState = 10;
                    break;

                case 10:
                    telemetry.addLine("🏁 AUTO COMPLETE!");
                    break;
            }
        }
    }

    private void manageIntakeTiming() {
        // Stop intake2 after 1.5 seconds during Path 2
        if (pathState == 1 && (System.currentTimeMillis() - stateStartTime) > 1500) {
            intake2.setPower(0);
        }
    }

    private void updateShooter(Pose robotPose) {
        // Calculate distance to goal
        double dx = GOAL_X - robotPose.getX();
        double dy = GOAL_Y - robotPose.getY();
        distanceToGoal = Math.sqrt(dx * dx + dy * dy);

        // Distance-based shooter velocity
        shooterSpeed = 6.85 * distanceToGoal + 1050;
        shooter.setVelocity(shooterSpeed);

        // Distance-based angle servo position
        if (distanceToGoal < 55) anglePos = 0.00;
        else if (distanceToGoal < 65) anglePos = 0.01;
        else if (distanceToGoal < 85) anglePos = 0.02;
        else if (distanceToGoal < 110) anglePos = 0.03;
        else if (distanceToGoal < 125) anglePos = 0.04;
        else anglePos = 0.06;

        angleServo.setPosition(anglePos);
    }

    private void updateTurret(Pose robotPose) {
        // Calculate angle to goal
        double dx = GOAL_X - robotPose.getX();
        double dy = GOAL_Y - robotPose.getY();
        double goalFieldHeading = Math.atan2(dy, dx) + TURRET_HEADING_OFFSET;
        double desiredTurretRad = AngleUnit.normalizeRadians(goalFieldHeading - robotPose.getHeading());

        // Convert to motor ticks and clamp
        int targetTicks = (int) Math.round(desiredTurretRad * AIM_TICKS_PER_RAD);
        targetTicks = Math.max(AIM_MIN_TICKS, Math.min(AIM_MAX_TICKS, targetTicks));

        aimMotor.setTargetPosition(targetTicks);
    }

    private void initHardware() {
        // Turret aim motor
        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aimMotor.setTargetPosition(0);
        aimMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        aimMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        aimMotor.setPower(AIM_POWER);

        // Shooter motor
        shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Shooter angle servo
        angleServo = hardwareMap.get(Servo.class, "shooterServo");
        angleServo.setPosition(0);

        // Intake motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake2.setDirection(DcMotor.Direction.REVERSE);
    }

    // Inner Paths class with YOUR ORIGINAL coordinates
    public class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(27.000, 130.000), new Pose(49.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(49.000, 60.000), new Pose(14.000, 60.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(14.000, 60.000), new Pose(72.000, 72.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(72.000, 72.000), new Pose(6.000, 65.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(6.000, 65.000), new Pose(72.000, 72.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(72.000, 72.000), new Pose(6.000, 65.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(6.000, 65.000), new Pose(72.000, 72.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(72.000, 72.000), new Pose(35.000, 84.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(35.000, 84.000), new Pose(14.000, 84.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(14.000, 84.000), new Pose(60.000, 84.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 84.000), new Pose(12.000, 70.000)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}