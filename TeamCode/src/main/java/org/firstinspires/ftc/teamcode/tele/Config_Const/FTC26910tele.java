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
            //gamepad1 base and front arm controll
            if (gamepad1.left_bumper) {
                robot.c.lowSpeed();
            }
            else {
                robot.c.normalSpeed();
            }
            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;
            robot.c.teleDrive(x, y, rx);
            robot.cla.frontarmp(-gamepad1.right_stick_y);
            robot.cla.updatefm();

            //gamepad2 arm control
            if (gamepad2.y) {
                robot.cla.take();
                robot.cla.gathering = true;
            }
            else{
                robot.cla.gathering = false;
            }
            if (gamepad2.a) {
                robot.cla.trans();
            }

            if (gamepad2.x)
                robot.cla.takes();
            if (gamepad2.b)
                robot.cla.drops();
            if (gamepad2.dpad_left)
                robot.cla.dump();
            if (gamepad2.dpad_right)
                robot.cla.endback();
            if (gamepad2.dpad_up||gamepad2.a){
                robot.cla.rev = true;
            }
            else {
                robot.cla.rev = false;
            }
            if (gamepad2.dpad_down)
                robot.cla.fback();


            if (gamepad2.left_bumper && -gamepad2.right_stick_y > 0.8)
                robot.cla.highbasket();
            else if (gamepad2.left_bumper && -gamepad2.right_stick_y < -0.8)
                robot.cla.lowbasket();
            else if (gamepad2.left_bumper && -gamepad2.right_stick_y > -0.2 && -gamepad2.left_stick_y < 0.2)
                robot.cla.mainback();
            else if (gamepad2.right_bumper && -gamepad2.right_stick_y > -0.2 && -gamepad2.left_stick_y < 0.2)
                robot.cla.spepos();
            else if (gamepad2.right_bumper && -gamepad2.right_stick_y  > 0.8)
                robot.cla.highbar();
            else if (gamepad2.right_bumper && -gamepad2.right_stick_y  < -0.8)
                robot.cla.lowbar();
        }
    }
}