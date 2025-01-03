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

    public Pose2d startPos = new Pose2d(0, 0, 0);
    public Pose2d highBarPos = new Pose2d(-28.7918, 5.8298, 0);////改
    public Pose2d g1Pos = new Pose2d(-16.693, -7.9348, Math.toRadians(240));
    public Pose2d g2Pos = new Pose2d(39.298, 5.125, Math.toRadians(122.795));//需要调整
    public Pose2d g3Pos = new Pose2d(35.05,23.511, Math.toRadians(151.99));
    //public Pose2d g3Pos = new Pose2d(-17.7793, 30.1948, 1.9523);//需要调整
    private double barOffsetX = .08;
    public Pose2d highBarPos1 = new Pose2d(3.818 - barOffsetX, -51.4428 - barOffsetX, Math.toRadians(121.259));//需要调整
    public Pose2d highBarPos2 = new Pose2d(0.2 - barOffsetX, 1.5 - barOffsetX, -0.1);//需要调整
    public Pose2d highBarPos3 = new Pose2d(-0.3 - barOffsetX, 0 - barOffsetX, 0);//需要调整

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        AutoRobot robot = new AutoRobot(hardwareMap);
        TrajectorySequence FinalAuto1 = robot.chassis.drive.trajectorySequenceBuilder(startPos)

                .lineToLinearHeading(highBarPos)//走到杆前面
                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                    robot.arm.highBar();//把arm伸上去
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.dropSpe();//挂上并松手
                })
                .waitSeconds(0.2)//操作等待时间
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.VtBack();//把arm收回来
                })



                .lineToLinearHeading(g1Pos)//准备吸取第一个地上的
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.arm.frontIntake();//放下滚吸
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.arm.getIntake=true;
                    robot.arm.intakeMupdate();//开启滚吸
                    robot.arm.HzArmSet(1500);//往前申滑轨
                })
                .waitSeconds(1)//操作时间
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.getIntake=false;
                    robot.arm.intakeMupdate();
                    robot.arm.HzArmSet(0);
                    robot.arm.inArmTrans();

                })
                .waitSeconds(0.5)
                .lineToLinearHeading(highBarPos1)//夹样本位置
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=true;
                    robot.arm.intakeMupdate();//停止滚吸
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=false;
                    robot.arm.intakeMupdate();//停止滚吸
                    robot.arm.frontIntake();
                    robot.arm.highBasket();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.basketOut();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=false;
                    robot.arm.intakeMupdate();//停止滚吸
                    robot.arm.basketBack();
                    robot.arm.VtBack();
                })
                //.waitSeconds(2)
                .build();

        TrajectorySequence FinalAuto2 = robot.chassis.drive.trajectorySequenceBuilder(startPos)

                .lineToLinearHeading(g2Pos)//准备吸取第2个地上的
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.arm.getIntake=true;
                    robot.arm.intakeMupdate();//开启滚吸
                    robot.arm.HzArmSet(1400);//往前申滑轨
                })
                .waitSeconds(0.8)//操作时间
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.getIntake=false;
                    robot.arm.intakeMupdate();
                    robot.arm.HzArmSet(0);
                    robot.arm.inArmTrans();

                })
                .waitSeconds(0.5)
                .lineToLinearHeading(highBarPos2)//夹样本位置
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=true;
                    robot.arm.intakeMupdate();//停止滚吸
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=false;
                    robot.arm.intakeMupdate();//停止滚吸
                    robot.arm.frontIntake();
                    robot.arm.highBasket();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.basketOut();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=false;
                    robot.arm.intakeMupdate();//停止滚吸
                    robot.arm.basketBack();
                    robot.arm.VtBack();
                })
                //.waitSeconds(2)
                .build();
        TrajectorySequence FinalAuto3 = robot.chassis.drive.trajectorySequenceBuilder(startPos)

                .lineToLinearHeading(g3Pos)//准备吸取第2个地上的
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.getIntake=true;
                    robot.arm.intakeMupdate();//开启滚吸
                    robot.arm.HzArmSet(800);//往前申滑轨
                })
                .waitSeconds(1.3)//操作时间
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.getIntake=false;
                    robot.arm.intakeMupdate();
                    robot.arm.HzArmSet(0);
                    robot.arm.inArmTrans();

                })
                .waitSeconds(0.5)
                .lineToLinearHeading(highBarPos3)//夹样本位置
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=true;
                    robot.arm.intakeMupdate();//停止滚吸
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=false;
                    robot.arm.intakeMupdate();//停止滚吸
                    robot.arm.frontIntake();
                    robot.arm.highBasket();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.basketOut();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=false;
                    robot.arm.intakeMupdate();//停止滚吸
                    robot.arm.basketBack();
                    robot.arm.VtBack();
                })
                .waitSeconds(2)
                .build();



        robot.Autoinit(hardwareMap);
        waitForStart();
        robot.chassis.drive.setPoseEstimate(startPos);
        //while (!isStopRequested() && opModeIsActive()){
        robot.chassis.drive.followTrajectorySequence(FinalAuto1);
        robot.chassis.drive.setPoseEstimate(new Pose2d(0,0,0));
        robot.chassis.drive.followTrajectorySequence(FinalAuto2);
        robot.chassis.drive.setPoseEstimate(new Pose2d(0,0,0));
        robot.chassis.drive.followTrajectorySequence(FinalAuto3);


    }

}
