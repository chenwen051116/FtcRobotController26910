//package org.firstinspires.ftc.teamcode.tests;
//
//import static org.firstinspires.ftc.teamcode.lib.ConfigsNConst.rx;
//import static org.firstinspires.ftc.teamcode.lib.ConfigsNConst.x;
//import static org.firstinspires.ftc.teamcode.lib.ConfigsNConst.y;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.lib.Chassis;
//import org.firstinspires.ftc.teamcode.lib.ShVarsKt;
//
//@TeleOp
//public class Teletest extends LinearOpMode {
//
//
//    @Override
//    public void runOpMode() {
//        ShVarsKt.shLinOp = this;
//        Chassis c = new Chassis(hardwareMap);
//        c.TeleInit(hardwareMap);
//        waitForStart();
//        while (opModeIsActive()) {
//            if (gamepad1.right_trigger > 0.1 || gamepad2.left_trigger > 0.1)
//                c.lowSpeed();
//            else
//                c.normalSpeed();
//            x = gamepad1.left_stick_x;
//            y = gamepad1.left_stick_y;
//            rx = gamepad1.right_stick_x;
//            c.teleDrive(x, y, rx);
//        }
//    }
//}
