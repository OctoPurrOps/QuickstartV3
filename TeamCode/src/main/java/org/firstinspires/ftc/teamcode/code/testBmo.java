package org.firstinspires.ftc.teamcode.code;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled

@Autonomous(name = "Blue Paths Auto", group = "Autonomous")
public class testBmo extends OpMode {

    // Pedro follower + timers
    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;

    // Paths holder
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;

        public Paths(Follower follower) {

            // Path 1: (-59,129) → (-50,113)
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(27.000, 130.000),

                                    new Pose(58.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 100.000),

                                    new Pose(30.000, 100.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(30.000, 100.000),

                                    new Pose(58.000, 100.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 100.000),

                                    new Pose(51.000, 77.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(51.000, 77.000),

                                    new Pose(30.000, 77.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(30.000, 77.000),

                                    new Pose(58.000, 100.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 100.000),

                                    new Pose(45.000, 56.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.000, 56.000),

                                    new Pose(30.000, 56.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(30.000, 56.000),

                                    new Pose(58.000, 100.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();





        }
    }

    private Paths paths;

    // Simple path state machine
    public enum PathState {
        PATH_1,
        PATH_2,
        PATH_3,
        PATH_4,
        PATH_5,
        PATH_6,
        PATH_7,
        PATH_8,
        PATH_9,
        PATH_10,
        PATH_11,




        DONE
    }

    private PathState pathState = PathState.PATH_1;

    private void setPathState(PathState newState) {
        pathState = newState;
        if (pathTimer != null) pathTimer.reset();
    }

    // Core FSM using follower.isBusy() pattern from Pedro docs
    private void updateStateMachine() {
        switch (pathState) {

            case PATH_1:
                // If not currently following anything, start Path1
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path1, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_2);

                }
                break;

            case PATH_2:
                // When Path1 finishes, start Path2
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_3);
                }
                break;
            case PATH_3:
                // When Path1 finishes, start Path2
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_4);
                }
                break;
            case PATH_4:
                // When Path1 finishes, start Path2
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_5);
                }
                break;
            case PATH_5:
                // When Path1 finishes, start Path2
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_6);
                }
                break;
            case PATH_6:
                // When Path1 finishes, start Path2
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_7);
                }
                break;
            case PATH_7:
                // When Path1 finishes, start Path2
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_8);
                }
                break;
            case PATH_8:
                // When Path1 finishes, start Path2
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_9);
                }
                break;
            case PATH_9:
                // When Path1 finishes, start Path2
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_10);
                }
                break;
            case PATH_10:
                // When Path1 finishes, start Path2
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.PATH_11);
                }
                break;


            case PATH_11:
                // When Path2 finishes, start Path3
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path11, true);
                    follower.setMaxPower(1.0);
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                // All paths completed
                // follower.update() still runs in loop; robot just holds pose
                break;
        }
    }

    @Override
    public void init() {
        try {
            // Timers
            pathTimer = new ElapsedTime();
            opModeTimer = new ElapsedTime();

            // Create follower from your Constants helper
            follower = Constants.createFollower(hardwareMap);
            // Start pose must match Path1 start
            follower.setPose(new Pose(27, 130, Math.toRadians(144)));

            // Build paths
            paths = new Paths(follower);

            telemetry.addLine("✅ BLUE PATHS AUTO READY (3 paths)");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("❌ Init Error: " + e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void start() {
        opModeTimer.reset();
        pathTimer.reset();
        setPathState(PathState.PATH_1);
    }

    @Override
    public void loop() {
        try {
            // Always update Pedro follower
            follower.update();

            // Run path FSM
            updateStateMachine();

            // Basic telemetry
            telemetry.addData("Alliance", "BLUE");
            telemetry.addData("State", pathState);
            telemetry.addData("Pose", follower.getPose());
            telemetry.addData("OpMode Time", opModeTimer.seconds());
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("Loop Error: " + e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        // Nothing special to stop; follower will just stop commanding motion
    }
}
