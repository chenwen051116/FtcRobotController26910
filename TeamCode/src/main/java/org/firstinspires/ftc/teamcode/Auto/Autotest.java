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

    public Pose2d startPos = new Pose2d(12, -61.7, Math.toRadians(270));
    public Pose2d highBarPos = new Pose2d(5.3, -33.9, Math.toRadians(270));
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        AutoRobot robot = new AutoRobot(hardwareMap);
        TrajectorySequence FinalAuto = robot.chassis.drive.trajectorySequenceBuilder(startPos)

                .lineToLinearHeading(highBarPos)

                .UNSTABLE_addTemporalMarkerOffset(-1.0, () -> {
                    robot.arm.highBar();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                robot.arm.dropSpe();
                })

                .build();

        waitForStart();
        robot.chassis.drive.followTrajectorySequence(FinalAuto);
        while (!isStopRequested() && opModeIsActive()) ;
    }

}
