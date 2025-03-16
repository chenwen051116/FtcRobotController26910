package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.lib.ShVarsKt.shTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.AutoRobot;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "TestBlockTracking")
public class TestBlockTracking extends LinearOpMode {
    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        shTelemetry = telemetry;
        AutoRobot robot = new AutoRobot(hardwareMap);
        waitForStart();
        robot.startVisionThread();
        telemetry.addLine("Vision thread started");
        telemetry.update();
        while(opModeIsActive()){
            telemetry.addLine("into while");
            if (gamepad1.left_bumper){
                telemetry.addLine("Tracking block");
                synchronized (robot.getBlockList()) {
                    robot.trackBlock(AutoRobot.Companion.getBlockAtCenter(robot.getBlockList()));
                }
            }
        }
    }
}
