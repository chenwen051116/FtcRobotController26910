package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ext.roadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class Autoleft extends LinearOpMode {
    public static double g1x = -9.03;
    public static double g1y = -37.09;
    public static double g1ang = 180;
    public static double g2x = -8.73;
    public static double g2y = -46.505;
    public static double g2ang = 180;
    public static double g3x = -34;
    public static double g3y = -36;
    public static double g3ang = 270;
    public static int arml1 = 730;
    public static int arml2 = 720;
    public static int arml3 = 0;

    public static double b1x = 0.76;
    public static double b1y = -48.58;
    public static double b1ang = 140;
    public static double b2x = 0.76;
    public static double b2y = -48.58;
    public static double b2ang = 140;
    public static double b3x = 0.76;
    public static double b3y = -48.58;
    public static double b3ang = 140;


    public Pose2d startPos = new Pose2d(0, 0, 0);
    public Pose2d highBarPos = new Pose2d(-29.7918, 10.8298, 0);

    //public Pose2d TurnPos = new Pose2d(-12.686, 24.763, 0);
    public Pose2d g1Pos = new Pose2d(g1x, g1y, Math.toRadians(g1ang));
    public Pose2d g2Pos = new Pose2d(g2x, g2y, Math.toRadians(g2ang));//需要调整
    public Pose2d g3Pos = new Pose2d(g3x, g3y, Math.toRadians(g3ang));//需要调整


    //public Pose2d speIntakePos =  new Pose2d(spein1x, spein1y, Math.toRadians(spein1ang));//需要调整
    //public Pose2d speIntakePos3 = new Pose2d(12, 0, 0);//需要调整
    //public Pose2d speIntakePos2 = new Pose2d(spein2x, spein2y, Math.toRadians(spein2ang));
    //public Pose2d speIntakePos4 = new Pose2d(12, 0, 0);//需要调整
    public Pose2d highBarPos1 = new Pose2d(b1x, b1y, Math.toRadians(b1ang));//需要调整
    public Pose2d highBarPos2 = new Pose2d(b2x, b2y, Math.toRadians(b2ang));//需要调整
    public Pose2d highBarPos3 = new Pose2d(b3x, b3y, Math.toRadians(b3ang));//需要调整

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        AutoRobot robot = new AutoRobot(hardwareMap);
        TrajectorySequence FinalAuto1 = robot.getChassis().drive.trajectorySequenceBuilder(startPos)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().highBar();//把arm伸上去
                    robot.getArm().speClaw.setPosition(0.15);
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(highBarPos)//走到杆前面

                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    robot.getArm().dropSpe();//挂上并松手
                })
                .waitSeconds(0.2)//操作等待时间
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().VtBack();//把arm收回来
                })


                .lineToLinearHeading(g1Pos)//准备吸取第一个地上的
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.getArm().HzArmSet(arml1);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.getArm().frontIntakeDown();
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().frontIntake();

                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().frontArmBack();

                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().inArmTrans();
                })
                .waitSeconds(1.5)
                .lineToLinearHeading(highBarPos1)
                //准备吸取第一个地上的

                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.getArm().highBasket();
                })


                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().basketOut();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().basketBack();
                    robot.getArm().frontIntakeDown();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(g2Pos)//准备吸取第一个地上的
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.getArm().HzArmSet(arml2);
                    robot.getArm().VtBack();
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.getArm().frontIntakeDown();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().frontIntake();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().frontArmBack();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().inArmTrans();
                })
                .waitSeconds(1.5)
                .lineToLinearHeading(highBarPos2)

                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.getArm().highBasket();
                })


                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().basketOut();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().basketBack();
                    robot.getArm().frontIntakeDown();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(g3Pos)//准备吸取第一个地上的
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.getArm().HzArmSet(arml3);
                    robot.getArm().VtBack();
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.getArm().frontIntakeDown();
                    robot.getArm().inTurn(1);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().frontIntake();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().frontArmBack();
                    robot.getArm().inTurn(0.5);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    robot.getArm().inArmTrans();

                })
                .waitSeconds(1.5)
                .lineToLinearHeading(highBarPos3)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.getArm().highBasket();
                })


                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().basketOut();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().basketBack();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.getArm().VtBack();//把arm收回来

                    robot.getArm().HzArmSet(0);
                })
                .build();



//
//        TrajectorySequence FinalAuto4 = robot.getChassis().drive.trajectorySequenceBuilder(startPos)
//
////                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
////                    robot.getArm().closeClaw();//关夹子
////                })
////                .waitSeconds(0.5)
////                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
////                    robot.getArm().highBar();//把arm伸上去
////                })
////                .lineToLinearHeading(highBarPos3)//放的位置
////                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
////                    robot.getArm().dropSpe();//挂上并松手
////                })
////                .waitSeconds(0.2)//操作等待时间
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.getArm().VtBack();//把arm收回来
//                    robot.getArm().frontIntake();
//                    robot.getArm().HzArmSet(0);
//                })
//
//                //lineToLinearHeading(speIntakePos4)//夹样本位置
//                //.waitSeconds(0.5)
//                .build();



        robot.Autoinit(hardwareMap);
        waitForStart();
        robot.getChassis().drive.setPoseEstimate(startPos);
        //while (!isStopRequested() && opModeIsActive()){
        robot.getChassis().drive.followTrajectorySequence(FinalAuto1);

    }

}
