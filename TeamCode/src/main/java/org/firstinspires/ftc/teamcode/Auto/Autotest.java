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
    public Pose2d g2Pos = new Pose2d(-14.9978, 24.2948, 2.1414);
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        AutoRobot robot = new AutoRobot(hardwareMap);
        TrajectorySequence FinalAuto = robot.chassis.drive.trajectorySequenceBuilder(startPos)

                .lineToLinearHeading(highBarPos)

                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                    robot.arm.highBar();
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                robot.arm.dropSpe();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.takeSpePos();
                })
                .lineToLinearHeading(g1Pos)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.frontIntake();
                    robot.arm.getIntake=true;
                    robot.arm.intakeMupdate();
                    robot.arm.HzArmSet(1500);
                })
                .waitSeconds(2)
                .lineToLinearHeading(g1tPos)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.arm.frontIntake();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=true;
                    robot.arm.intakeMupdate();
                    robot.arm.HzArmSet(1200);
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    robot.arm.getIntake=true;
                    robot.arm.reverseIntake=false;
                    robot.arm.intakeMupdate();
                    robot.arm.HzArmSet(100);
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(g2Pos)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=true;
                    robot.arm.intakeMupdate();
                    robot.arm.HzArmSet(1200);
                })
                .waitSeconds(1)
                .lineToLinearHeading(g1tPos)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=true;
                    robot.arm.intakeMupdate();
                    robot.arm.HzArmSet(1200);
                })
                .waitSeconds(2)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.inArmTrans();
                    robot.arm.getIntake=false;
                    robot.arm.reverseIntake=false;
                    robot.arm.intakeMupdate();
                    robot.arm.HzArmSet(0);
                })
                .waitSeconds(2)
                .build();
        robot.Autoinit(hardwareMap);
        waitForStart();
        robot.chassis.drive.setPoseEstimate(startPos);
        //while (!isStopRequested() && opModeIsActive()){
            robot.chassis.drive.followTrajectorySequence(FinalAuto);
        //}

    }

}
