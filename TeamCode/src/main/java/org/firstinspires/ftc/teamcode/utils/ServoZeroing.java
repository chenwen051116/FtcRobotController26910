package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.Robot;

@Autonomous
public class ServoZeroing extends LinearOpMode {


    @Override
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);
        robot.Teleinit(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            robot.arm.speClaw.setPosition(0);
        }
    }
}

