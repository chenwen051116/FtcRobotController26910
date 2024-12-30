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
//@Autonomous
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class Autoright2 extends LinearOpMode {

    public Pose2d startPos = new Pose2d(0, 0, 0);
    public Pose2d highBarPos = new Pose2d(-29.7918, -5.8298, 0);

    public Pose2d TurnPos = new Pose2d(-29.7918, -5.8298, 0);
    public Pose2d g1Pos = new Pose2d(-16.693, 7.9348, 2.0572);
    public Pose2d g2Pos = new Pose2d(-14.9978, 24.2948, 2.1514);//需要调整

    public Pose2d g3Pos = new Pose2d(-14.9978, 24.2948, 2.1514);//需要调整


    public Pose2d speIntakePos = new Pose2d(12, 40, 3.09159);//需要调整
    public Pose2d speIntakePos3 = new Pose2d(-3.98, 0, 6.08);//需要调整
    public Pose2d speIntakePos2 = new Pose2d(-2, -1, 0.5);//需要调整
    public Pose2d speIntakePos4 = new Pose2d(-1, 0, 0.5);//需要调整
    public Pose2d highBarPos1 = new Pose2d(41.852, 55.178, 3.15159);//需要调整
    public Pose2d highBarPos2 = new Pose2d(34.152, 65.178, 3.15159);//需要调整
    public Pose2d highBarPos3 = new Pose2d(38.552, 67.178, 3.15159);//需要调整

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        AutoRobot robot = new AutoRobot(hardwareMap);
        TrajectorySequence FinalAuto1 = robot.chassis.drive.trajectorySequenceBuilder(startPos)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.highBar();//把arm伸上去
                })
                .waitSeconds(1)
                .lineToLinearHeading(highBarPos)//走到杆前面

                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    robot.arm.dropSpe();//挂上并松手
                })
                .waitSeconds(0.2)//操作等待时间
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.VtBack();//把arm收回来
                })


                .lineToLinearHeading(TurnPos)//准备吸取第一个地上的
                .lineToLinearHeading(g1Pos)//准备吸取第一个地上的
                .strafeRight(1)
                .back(10)
                .lineToLinearHeading(TurnPos)//准备吸取第一个地上的
                .lineToLinearHeading(g2Pos)//准备吸取第一个地上的
                .strafeRight(1)
                .back(10)
                .lineToLinearHeading(TurnPos)//准备吸取第一个地上的
                .lineToLinearHeading(g3Pos)//准备吸取第一个地上的
                .strafeRight(1)
                .back(10)


                .lineToLinearHeading(speIntakePos)//夹样本位置
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(-1.8, () -> {
                    robot.arm.HzArmSet(100);//收回前滑轨
                    robot.arm.inArmTrans();
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=false;
                    robot.arm.intakeMupdate();//停止滚吸
                    robot.arm.dropSpe();//打开夹子
                    robot.arm.takeSpePos();//夹样本高度
                })
                .waitSeconds(0.5)
                .build();

        TrajectorySequence FinalAuto2 = robot.chassis.drive.trajectorySequenceBuilder(startPos)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.closeClaw();//关夹子
                    //robot.arm.HzArmSet(100);//收回前滑轨
                    robot.arm.inArmTrans();
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=false;
                    robot.arm.intakeMupdate();//停止滚吸

                })
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    robot.arm.highBar();//把arm伸上去
                })
                .lineToLinearHeading(highBarPos1)//放的位置

                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    robot.arm.dropSpe();//挂上并松手
                })
                .waitSeconds(0.2)//操作等待时间
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.takeSpePos();//把arm收回来
                })

                .lineToLinearHeading(speIntakePos3)//夹样本位置
                //.waitSeconds(0.5)
                .build();

        TrajectorySequence FinalAuto3 = robot.chassis.drive.trajectorySequenceBuilder(startPos)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.closeClaw();//关夹子
                })
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    robot.arm.highBar();//把arm伸上去
                })
                .lineToLinearHeading(highBarPos2)//放的位置
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    robot.arm.dropSpe();//挂上并松手
                    robot.arm.frontIntake();
                })
                .waitSeconds(0.2)//操作等待时间
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {
                //    robot.arm.takeSpePos();//把arm收回来
                //})

                //.lineToLinearHeading(speIntakePos2)//夹样本位置
                //.waitSeconds(0.5)
                .forward(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.VtBack();//把arm收回来

                    robot.arm.HzArmSet(0);
                })
                .build();


//
//        TrajectorySequence FinalAuto4 = robot.chassis.drive.trajectorySequenceBuilder(startPos)
//
////                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
////                    robot.arm.closeClaw();//关夹子
////                })
////                .waitSeconds(0.5)
////                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
////                    robot.arm.highBar();//把arm伸上去
////                })
////                .lineToLinearHeading(highBarPos3)//放的位置
////                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
////                    robot.arm.dropSpe();//挂上并松手
////                })
////                .waitSeconds(0.2)//操作等待时间
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.arm.VtBack();//把arm收回来
//                    robot.arm.frontIntake();
//                    robot.arm.HzArmSet(0);
//                })
//
//                //lineToLinearHeading(speIntakePos4)//夹样本位置
//                //.waitSeconds(0.5)
//                .build();



        robot.Autoinit(hardwareMap);
        waitForStart();
        robot.chassis.drive.setPoseEstimate(startPos);
        //while (!isStopRequested() && opModeIsActive()){
            robot.chassis.drive.followTrajectorySequence(FinalAuto1);
            robot.chassis.drive.setPoseEstimate(new Pose2d(0,0,0));
        robot.chassis.drive.followTrajectorySequence(FinalAuto2);
        robot.chassis.drive.setPoseEstimate(new Pose2d(0,0,0));
        robot.chassis.drive.followTrajectorySequence(FinalAuto3);
        //robot.chassis.drive.setPoseEstimate(new Pose2d(0,0,0));
        //robot.chassis.drive.followTrajectorySequence(FinalAuto4);

    }

}
