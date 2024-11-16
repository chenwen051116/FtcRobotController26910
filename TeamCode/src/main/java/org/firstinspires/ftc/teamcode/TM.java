package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.util.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "TM")
public class TM extends LinearOpMode {
    private DcMotor m1;
    //PIDController control = new PIDController(0.05,0.0,0.1);


    @Override
    public void runOpMode() {
        m1 = hardwareMap.get(DcMotor.class, "m");
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
            m1.setPower(0.5);
            //m1.setPower(gamepad1.left_stick_y);
        }
    }
}
