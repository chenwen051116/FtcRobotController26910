package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ext.roadrunner.util.PIDController;


@TeleOp(name = "Testmotor")
public class TestMotor extends LinearOpMode {
    PIDController control = new PIDController(0.05, 0.0, 0.1);
    private DcMotor m1;

    @Override
    public void runOpMode() {
        m1 = hardwareMap.get(DcMotor.class, "m3");
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        int targetPosition = 100;
        while (opModeIsActive()) {
            //control.setInput(m1.getCurrentPosition());
            control.setSetpoint(targetPosition);
            double command = control.performPID(m1.getCurrentPosition());
            // assign motor the PID output
            m1.setPower(command);
            //m1.setPower(gamepad1.left_stick_y);
        }
    }
}
