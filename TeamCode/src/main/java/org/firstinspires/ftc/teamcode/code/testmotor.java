package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled

@TeleOp(name = "ManualEncoderCheck", group = "Test")
public class testmotor extends LinearOpMode {

    private DcMotor motor;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "aimMotor"); // change to your motor name
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // zero at start[web:13]
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);     // free, but still reads ticks[web:13]

        telemetry.addLine("Ready. Rotate motor by hand.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            int pos = motor.getCurrentPosition();               // current ticks[web:7]
            telemetry.addData("Encoder ticks", pos);
            telemetry.update();
        }
    }
}
