package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ManualEncoderSpinTest", group = "Test")
public class test extends LinearOpMode {

    private DcMotorEx shooter;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // CHANGE name to your shooter motor config name
        shooter = hardwareMap.get(DcMotorEx.class, "motor.");

        // Reset and enable encoder counting
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Very important: NO power set anywhere
        shooter.setPower(0);

        telemetry.addLine("ManualEncoderSpinTest READY");
        telemetry.addLine("After start, turn the motor shaft by hand and watch position.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        timer.reset();

        while (opModeIsActive()) {
            int pos = shooter.getCurrentPosition();   // encoder ticks [web:128][web:127]

            telemetry.addData("Time", timer.seconds());
            telemetry.addData("Encoder Pos", pos);
            telemetry.update();

            // small delay so telemetry is readable
            sleep(50);
        }
    }
}
