package org.firstinspires.ftc.teamcode.code;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled

@TeleOp(name = "PIDF-Teleop", group = "teleop")
@Configurable
public class pidf extends OpMode {
    DcMotorEx shooter;
    DcMotor intake;
    DcMotor intake2;

    double high=1300, low=1900;
    double targetVelocity=high;
    double P=0, F=0;

    double[] stepSizes={10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex=1;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients=new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("init completed");

        intake.setPower(0.8);
        intake2.setPower(0.8);
    }

    @Override
    public void loop() {
        if(gamepad1.yWasPressed()) {
            if(targetVelocity==high) {
                targetVelocity=low;
            }
            else {
                targetVelocity=high;
            }
        }
        if(gamepad1.bWasPressed()) {
            stepIndex=(stepIndex+1)%stepSizes.length;
        }

        if(gamepad1.dpadRightWasPressed()) {
            F+=stepSizes[stepIndex];
        }
        if(gamepad1.dpadLeftWasPressed()) {
            F-=stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed()) {
            P+=stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()) {
            P-=stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients=new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter.setVelocity(targetVelocity);
        double velocity=shooter.getVelocity();
        double error=targetVelocity-velocity;

        telemetry.addData("Target Velocity: ", targetVelocity);
        telemetry.addData("Velocity: ", "%.2f", velocity);
        telemetry.addData("Error: ", "%.2f", error);
        telemetry.addLine("-------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad R/L)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.update();
    }
}