package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.Robot;
import org.firstinspires.ftc.teamcode.lib.schedule.Scheduler;

@Autonomous
public class ServoZeroing extends LinearOpMode {


    @Override
    public void runOpMode(){
        Scheduler scheduler = new Scheduler();
        Robot robot = new Robot(hardwareMap, scheduler);
        robot.teleInit(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()){
            robot.arm.speClaw.setPosition(0);
        }
    }
}

