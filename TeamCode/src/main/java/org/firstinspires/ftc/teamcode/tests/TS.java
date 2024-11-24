package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "TS")
public class TS extends LinearOpMode {
    public Servo s = null;
    //PIDController control = new PIDController(0.05,0.0,0.1);


    @Override
    public void runOpMode() {
        s = hardwareMap.get(Servo.class, "s");
        //m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        //int targetPosition = 100;
        while (opModeIsActive()) {
            //control.setInput(m1.getCurrentPosition());
            //control.setSetpoint(targetPosition);
            //double command = control.performPID(m1.getCurrentPosition());
            // assign motor the PID output
            if (gamepad1.y)
                s.setPosition(0.5);
            else if (gamepad1.a)
                s.setPosition(1);
            else {
                s.setPosition(0);
            }
            //m1.setPower(gamepad1.left_stick_y);
        }
    }
}
