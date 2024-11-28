package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Testencoder")
public class Testencoder extends LinearOpMode {
    private DcMotor m2;


    @Override
    public void runOpMode() {
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            //control.setInput(m1.getCurrentPosition());
            telemetry.addData("position:", m2.getCurrentPosition());
            telemetry.update();
        }
    }
}
