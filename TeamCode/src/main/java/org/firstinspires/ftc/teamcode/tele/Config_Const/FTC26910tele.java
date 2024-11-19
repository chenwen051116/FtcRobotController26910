package org.firstinspires.ftc.teamcode.Tele.Config_Const;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Tele.Config_Const.Robot;

import static org.firstinspires.ftc.teamcode.Tele.Config_Const.ConfigsNConst.rx;
import static org.firstinspires.ftc.teamcode.Tele.Config_Const.ConfigsNConst.x;
import static org.firstinspires.ftc.teamcode.Tele.Config_Const.ConfigsNConst.y;

@TeleOp
public class FTC26910tele extends LinearOpMode {
    private Pose2d endpos = new Pose2d(-34.9, 56.9, Math.toRadians(270));


    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, endpos);
        robot.Teleinit(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a)
                robot.c.lowSpeed();
            else
                robot.c.normalSpeed();


            if (gamepad2.y) {
                robot.cla.take();
                robot.cla.gathering = true;
            }
            else{
                robot.cla.gathering = false;
            }
            if (gamepad2.a)
                robot.cla.trans();
            if (gamepad2.dpad_down)
                robot.cla.reve();
            if (!gamepad2.dpad_down) {
                robot.cla.rollback();
            }

            if (gamepad2.dpad_up && -gamepad2.left_stick_y > 0.8)
                robot.cla.highbasket();
            else if (gamepad2.dpad_up && -gamepad2.left_stick_y > -0.2 && -gamepad2.left_stick_y < 0.2)
                robot.cla.mainback();
//            else if (gamepad2.left_trigger > 0.1 && -gamepad2.left_stick_y>0.8)
//                robot.cla.highbar();
//            else if (gamepad2.left_trigger > 0.1 && gamepad2.dpad_up)
//                robot.cla.highbasket();
//            else if (gamepad2.left_trigger > 0.1 && gamepad2.dpad_down)
//                robot.cla.lowbasket();
//            else if (gamepad2.left_trigger > 0.1 && gamepad2.dpad_left)
//                robot.cla.spepos();
            else if (gamepad2.dpad_up && -gamepad2.left_stick_y < -0.8)
                robot.cla.lowbasket();
            else if (gamepad2.dpad_up && gamepad2.left_stick_x < -0.8)
                robot.cla.spepos();
            else if (gamepad2.dpad_up && gamepad2.left_stick_x > 0.8)
                robot.cla.highbar();

            if (gamepad2.x)
                robot.cla.takes();
            if (gamepad2.b)
                robot.cla.drops();
            if (gamepad2.dpad_left)
                robot.cla.dump();
            if (gamepad2.dpad_right)
                robot.cla.endback();
            robot.cla.frontarmp(-gamepad2.right_stick_y);
            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;
            if (gamepad1.left_bumper){
                rx = -0.8;
            }
            else if (gamepad1.right_bumper){
                rx = 0.8;
            }

            if (gamepad1.left_trigger>0.5){
                rx = -0.3;
            }
            else if (gamepad1.right_trigger>0.5) {
                rx = 0.3;
            }
            robot.cla.updatefm();
            robot.c.teleDrive(x, y, rx);
        }
    }
}