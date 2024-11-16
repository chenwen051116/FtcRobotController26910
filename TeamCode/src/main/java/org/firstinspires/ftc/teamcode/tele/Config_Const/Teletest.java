package org.firstinspires.ftc.teamcode.Tele.Config_Const;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.drive.SampleMecanumDrive;

import java.util.Timer;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Tele.Config_Const.Robot;

import static org.firstinspires.ftc.teamcode.Tele.Config_Const.ConfigsNConst.rx;
import static org.firstinspires.ftc.teamcode.Tele.Config_Const.ConfigsNConst.x;
import static org.firstinspires.ftc.teamcode.Tele.Config_Const.ConfigsNConst.y;

@TeleOp
public class Teletest extends LinearOpMode{
    private Pose2d endpose = new Pose2d(-34.9, 56.9, Math.toRadians(270));


    @Override
    public void runOpMode() {

        Chassis c = new Chassis(hardwareMap, endpose);
        c.TeleInit(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.1 || gamepad2.left_trigger > 0.1)
                c.lowSpeed();
            else
                c.normalSpeed();
            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;
            c.teleDrive(x, y, rx);
        }
    }
}
