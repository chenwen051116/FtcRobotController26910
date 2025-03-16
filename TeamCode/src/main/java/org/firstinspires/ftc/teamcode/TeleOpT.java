package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.lib.ConfigsNConst.rx;
import static org.firstinspires.ftc.teamcode.lib.ConfigsNConst.x;
import static org.firstinspires.ftc.teamcode.lib.ConfigsNConst.y;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.Robot;
import org.firstinspires.ftc.teamcode.lib.schedule.Scheduler;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpT extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler scheduler = new Scheduler();
        Robot robot = new Robot(hardwareMap, scheduler);
        robot.teleInit(hardwareMap, telemetry);

        double intake_rotate = 0.0;

        waitForStart();
        Thread visionThread = robot.startVisionThread();
        while (opModeIsActive()) {
            // check and execute all scheduled task at the beginning of each loop
            scheduler.elapse();

            //gamepad1 base and front arm controll
            if (gamepad1.left_bumper) {
                robot.chassis.lowSpeed();
            }
            else if(gamepad1.right_bumper){
                robot.chassis.lowlowSpeed();
            }
            else {
                robot.chassis.normalSpeed();
            }
            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;
            robot.chassis.teleDrive(x, y, rx);
            if (gamepad1.y) {
                robot.chassis.goOrigin();
            }
            if (gamepad1.a) {
                robot.chassis.setOrigin();
            }
            if(gamepad1.x){
                robot.barAuto();
            }
            if(gamepad1.b){
                robot.barbackAuto();
            }
            robot.chassis.cancelAuto();
//            telemetry.addData("x:", robot.chassis.lastpos.getX());
//            telemetry.addData("Y:", robot.chassis.lastpos.getY());
//            telemetry.addData("A:", robot.chassis.lastpos.getHeading());

            robot.arm.HzArmVel(-gamepad2.left_stick_y);
            //robot.arm.intakeMupdate();
            //telemetry.addData("imuheading", robot.chassis.drive.getimuExternalHeading());
//            telemetry.addData("position2:", robot.arm.VtRight.getCurrentPosition());
//            telemetry.addData("position1:", robot.arm.VtLeft.getPower());
//            telemetry.addData("position2:", robot.arm.VtRight.getPower());
            //telemetry.addData("HLangle", robot.v.hlGetangle(robot.v.getBlock(1)));
            //telemetry.addData("Block", "id=" + robot.v.getBlock(1).id + " size: " + robot.v.getBlock(1).width + "x" + robot.v.getBlock(1).height + " position: " + robot.v.getBlock(1).x + "," + robot.v.getBlock(1).y);

            if (gamepad2.left_trigger > 0.5) {
                intake_rotate = gamepad2.left_stick_x;
                robot.arm.switchinTurn();
            }
//            if (gamepad2.right_trigger > 0.5) {
//                telemetry.addData("controller", robot.v.getBlkAngAsRed());
//                telemetry.update();
//                intake_rotate = robot.v.getClawAngFromBlkAng(robot.v.getBlkAngAsRed());
//                robot.arm.inTurn(intake_rotate);
//            }

//            telemetry.addData("Rect_detected: ", robot.v.hlGetAngle(robot.v.getBlockNear()));
//            telemetry.addData("Actual angle: ", intake_rotate);

            if (gamepad2.dpad_down&&-gamepad2.right_stick_y <-0.5) {
                //robot.arm.frontIntake();
                robot.arm.vtArmlowreset();
            }
            if (gamepad2.dpad_down&&-gamepad2.right_stick_y > 0.5) {
                //robot.arm.frontIntake();
                robot.arm.vtArmreset();
            }
            if(gamepad2.dpad_up){
                robot.arm.releaseC();
            }
            if (gamepad2.dpad_up && (-gamepad2.right_stick_y > 0.5)) {
                robot.arm.finalClimb();
            }

            //gamepad2 arm control
            //front intake control
            if (gamepad2.y) {
                //robot.arm.frontIntake();
                if(gamepad2.right_trigger>0.5){
                    robot.arm.frontIntakeDownSlow();
                }
                else {
                    robot.arm.frontIntakeDownquick();
                }

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
            //robot.arm.reverseIntake = gamepad2.dpad_up;

            //quick horizontal slide back
//            if (gamepad2.dpad_down)
//                robot.arm.frontArmBack();
            //robot.arm.frontIntake();
            //robot.arm.getIntake = gamepad2.dpad_down || gamepad2.y;
            //all position of vertical arm
            if (gamepad2.left_bumper && -gamepad2.right_stick_y > 0.8)
                robot.arm.highBasket();
            else if (gamepad2.left_bumper && -gamepad2.right_stick_y < -0.8)
                robot.arm.lowBasket();
            else if (gamepad2.left_bumper && -gamepad2.right_stick_y > -0.2 && -gamepad2.right_stick_y < 0.2)
                robot.arm.VtBack();
            else if (gamepad2.right_bumper && -gamepad2.right_stick_y > -0.2 && -gamepad2.right_stick_y < 0.2)
                robot.arm.takeSpePos();
            else if (gamepad2.right_bumper && -gamepad2.right_stick_y > 0.8)
                robot.arm.highBar();
            else if (gamepad2.right_bumper && -gamepad2.right_stick_y < -0.8)
                robot.arm.lowBar();

            if(gamepad2.right_stick_button) {
                robot.alignClaw();
            }

//            telemetry.update();
        }
        visionThread.join();
    }
}