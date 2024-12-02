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
public class Autotest extends LinearOpMode {

    public Pose2d startPos = new Pose2d(0, 0, 0);
    public Pose2d highBarPos = new Pose2d(-28.4918, -1.8298, 0);
    public Pose2d g1Pos = new Pose2d(-16.693, 7.9348, 2.1372);
    public Pose2d g1tPos = new Pose2d(-15.693, 14.9348, 0.8477);
    public Pose2d g2Pos = new Pose2d(-14.9978, 24.2948, 2.1414);//需要调整

    public Pose2d g3Pos = new Pose2d(-14.9978, 24.2948, 2.1414);//需要调整
    public Pose2d speIntakePos = new Pose2d(-14.9978, 24.2948, 2.1414);//需要调整

    public Pose2d highBarPos1 = new Pose2d(-28.4918, -2.8298, 0);//需要调整
    public Pose2d highBarPos2 = new Pose2d(-28.4918, -3.8298, 0);//需要调整
    public Pose2d highBarPos3 = new Pose2d(-28.4918, -4.8298, 0);//需要调整
    public Pose2d highBarPos4 = new Pose2d(-28.4918, -5.8298, 0);//需要调整

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        AutoRobot robot = new AutoRobot(hardwareMap);
        TrajectorySequence FinalAuto = robot.chassis.drive.trajectorySequenceBuilder(startPos)

                .lineToLinearHeading(highBarPos)//走到杆前面
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                    robot.arm.highBar();//把arm伸上去
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                robot.arm.dropSpe();//挂上并松手
                })
                .waitSeconds(0.5)//操作等待时间
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.VtBack();//把arm收回来
                })



                .lineToLinearHeading(g1Pos)//准备吸取第一个地上的
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.arm.frontIntake();//放下滚吸
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.getIntake=true;
                    robot.arm.intakeMupdate();//开启滚吸
                    robot.arm.HzArmSet(1500);//往前申滑轨
                })
                .waitSeconds(1)//操作时间



                .lineToLinearHeading(g1tPos)//转身准备放
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=true;
                    robot.arm.intakeMupdate();//反转滚吸
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.getIntake=true;
                    robot.arm.reverseIntake=false;
                    robot.arm.intakeMupdate();//正转滚吸
                    robot.arm.HzArmSet(100);//收回前滑轨
                })
                .waitSeconds(0.5)



                .lineToLinearHeading(g2Pos)//第二个地上的吸取位置
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.HzArmSet(1500);
                })
                .waitSeconds(1)
                .lineToLinearHeading(g1tPos)//放回同样位置
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=true;
                    robot.arm.intakeMupdate();//反转电机
                    robot.arm.HzArmSet(1500);
                })
                .waitSeconds(1)
//要测的地方开始
/*
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.getIntake=true;
                    robot.arm.reverseIntake=false;
                    robot.arm.intakeMupdate();//正转滚吸
                    robot.arm.HzArmSet(100);//收回前滑轨
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(g3Pos)//第三个地上的位置
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.HzArmSet(1500);//伸长arm
                })
                .lineToLinearHeading(g1tPos)//放回同样位置
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=true;
                    robot.arm.intakeMupdate();//反转电机
                    robot.arm.HzArmSet(1500);
                })
                .waitSeconds(1)



                .lineToLinearHeading(speIntakePos)//夹样本位置
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.arm.inArmTrans();
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=false;
                    robot.arm.intakeMupdate();//停止滚吸
                    robot.arm.HzArmSet(100);//收回前滑轨
                    robot.arm.dropSpe();//打开夹子
                    robot.arm.takeSpePos();//夹样本高度
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.closeClaw();//关夹子
                })
                .lineToLinearHeading(highBarPos1)//放的位置
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                    robot.arm.highBar();//把arm伸上去
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.dropSpe();//挂上并松手
                })
                .waitSeconds(0.5)//操作等待时间
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.takeSpePos();//把arm收回来
                })



                .lineToLinearHeading(speIntakePos)//夹样本位置
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.arm.dropSpe();//打开夹子
                    robot.arm.takeSpePos();//夹样本高度
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.closeClaw();//关夹子
                })
                .lineToLinearHeading(highBarPos2)//放的位置
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                    robot.arm.highBar();//把arm伸上去
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.dropSpe();//挂上并松手
                })
                .waitSeconds(0.5)//操作等待时间
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.takeSpePos();//把arm收回来
                })





                .lineToLinearHeading(speIntakePos)//夹样本位置
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.arm.dropSpe();//打开夹子
                    robot.arm.takeSpePos();//夹样本高度
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.closeClaw();//关夹子
                })
                .lineToLinearHeading(highBarPos3)//放的位置
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                    robot.arm.highBar();//把arm伸上去
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.dropSpe();//挂上并松手
                })
                .waitSeconds(0.5)//操作等待时间
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.takeSpePos();//把arm收回来
                })


                .lineToLinearHeading(speIntakePos)//回到夹样本位置park
                 */


                //下面的是结束时收滑轨和舵机动作，一定保留
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.inArmTrans();
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=false;
                    robot.arm.intakeMupdate();
                    robot.arm.HzArmSet(200);
                    robot.arm.VtBack();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.HzArmSet(0);
                })
                .waitSeconds(1)
                .build();
        robot.Autoinit(hardwareMap);
        waitForStart();
        robot.chassis.drive.setPoseEstimate(startPos);
        //while (!isStopRequested() && opModeIsActive()){
            robot.chassis.drive.followTrajectorySequence(FinalAuto);
        //}

    }

}
