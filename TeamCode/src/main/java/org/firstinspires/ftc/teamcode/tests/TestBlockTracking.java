package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.AutoRobot;

@TeleOp(name = "TestBlockTracking")
public class TestBlockTracking extends LinearOpMode {
    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        AutoRobot robot = new AutoRobot(hardwareMap);

        waitForStart();
        robot.startVisionThread();
        while(opModeIsActive()){
            if (gamepad1.left_bumper){
                synchronized (robot.getBlockList()) {
                    robot.trackBlock(AutoRobot.Companion.getBlockAtCenter(robot.getBlockList()));
                }
            }
        }
    }
}
