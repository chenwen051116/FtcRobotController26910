package org.firstinspires.ftc.teamcode.tele.Config_Const;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tele.Config_Const.Robot;

import static org.firstinspires.ftc.teamcode.tele.Config_Const.ConfigsNConst.rx;
import static org.firstinspires.ftc.teamcode.tele.Config_Const.ConfigsNConst.x;
import static org.firstinspires.ftc.teamcode.tele.Config_Const.ConfigsNConst.y;

@TeleOp
public class FTC26910tele extends LinearOpMode {
    private Pose2d endpos = new Pose2d(-34.9, 56.9, Math.toRadians(270));
    private Robot robot = new Robot(hardwareMap, endpos);

    @Override
    public void runOpMode() {
        robot.Teleinit(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.1 || gamepad2.left_trigger > 0.1)
                robot.c.lowSpeed();
            else
                robot.c.normalSpeed();


            if (gamepad2.y)
                robot.cla.take();
            if (gamepad2.a)
                robot.cla.trans();
            if (gamepad2.right_trigger > 0.1)
                robot.cla.reve();
            if (gamepad2.right_trigger < 0.1){
                robot.cla.rollback();
            }

            if (gamepad2.left_trigger > 0.1 && gamepad2.left_stick_y>0.8)
                robot.cla.highbasket();
            else if (gamepad2.left_trigger > 0.1 && gamepad2.left_stick_y == 0)
                robot.cla.lowbar();
            else if (gamepad2.left_trigger > 0.1 && gamepad2.left_stick_y>0.8)
                robot.cla.highbar();
            else if (gamepad2.left_trigger > 0.1 && gamepad2.dpad_up)
                robot.cla.highbasket();
            else if (gamepad2.left_trigger > 0.1 && gamepad2.dpad_down)
                robot.cla.lowbasket();
            else if (gamepad2.left_trigger > 0.1 && gamepad2.dpad_left)
                robot.cla.spepos();
            else if (gamepad2.left_trigger > 0.1 && gamepad2.left_stick_y<-0.8)
                robot.cla.mainback();
            if (gamepad2.x)
                robot.cla.takes();
            if (gamepad2.b)
                robot.cla.drops();
            if (gamepad2.dpad_left)
                robot.cla.dump();
            if (gamepad2.dpad_right)
                robot.cla.endback();
            robot.cla.frontarmp(gamepad2.right_stick_y);
            robot.c.teleDrive(x, y, rx);
        }
    }
}