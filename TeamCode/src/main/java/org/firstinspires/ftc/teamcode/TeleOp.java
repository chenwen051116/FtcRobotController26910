package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.lib.ConfigsNConst.rx;
import static org.firstinspires.ftc.teamcode.lib.ConfigsNConst.x;
import static org.firstinspires.ftc.teamcode.lib.ConfigsNConst.y;

import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    private Pose2d endpos = new Pose2d(-34.9, 56.9, Math.toRadians(270));


    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, endpos);
        robot.Teleinit(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            //gamepad1 base and front arm controll
            if (gamepad1.left_bumper|| gamepad1.y) {
                robot.chassis.lowSpeed();
            }
            else {
                robot.chassis.normalSpeed();
            }
            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;
            robot.chassis.teleDrive(x, y, rx);
            robot.arm.HzArmVel(-gamepad1.right_stick_y);
            robot.arm.intakeMupdate();
            if (gamepad1.a) {
                //robot.arm.frontIntake();
                robot.arm.frontArmBack();
            }

            //gamepad2 arm control
            //front intake control
            if (gamepad2.y) {
                robot.arm.frontIntake();
            }
            if (gamepad2.a) {
                robot.arm.inArmTrans();
            }

            //claw control
            if (gamepad2.x)
                robot.arm.closeClaw();
            if (gamepad2.b)
                robot.arm.dropSpe();

            //basket output control
            if (gamepad2.dpad_left)
                robot.arm.basketOut();
            if (gamepad2.dpad_right)
                robot.arm.basketBack();

            //force reverse intake motor
            if (gamepad2.dpad_up){
                robot.arm.reverseIntake = true;
            }
            else {
                robot.arm.reverseIntake = false;
            }

            //quick horizontal slide back
//            if (gamepad2.dpad_down)
//                robot.arm.frontArmBack();
            if (gamepad2.dpad_down||gamepad2.y) {
                //robot.arm.frontIntake();
                robot.arm.getIntake = true;
            }
            else{
                robot.arm.getIntake = false;
            }
            //all position of vertical arm
            if (gamepad2.left_bumper && -gamepad2.right_stick_y > 0.8)
                robot.arm.highBasket();
            else if (gamepad2.left_bumper && -gamepad2.right_stick_y < -0.8)
                robot.arm.lowBasket();
            else if (gamepad2.left_bumper && -gamepad2.right_stick_y > -0.2 && -gamepad2.right_stick_y < 0.2)
                robot.arm.VtBack();
            else if (gamepad2.right_bumper && -gamepad2.right_stick_y > -0.2 && -gamepad2.right_stick_y < 0.2)
                robot.arm.takeSpePos();
            else if (gamepad2.right_bumper && -gamepad2.right_stick_y  > 0.5)
                robot.arm.highBar();
            else if (gamepad2.right_bumper && -gamepad2.right_stick_y  < -0.8)
                robot.arm.lowBar();
        }
    }
}