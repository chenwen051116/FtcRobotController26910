package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.ext.roadrunner.drive.SampleMecanumDrive;

public class AutoChassis {
    public double kp = 1;
    HardwareMap hardwareMap;
    SampleMecanumDrive drive;
    public Pose2d lastpos;
    public boolean isAuto = false;
    private Trajectory trajectory1;
    public AutoChassis(HardwareMap mp, Pose2d endpos) {
        this.hardwareMap = mp;
        this.drive = new SampleMecanumDrive(mp);
        this.drive.setPoseEstimate(endpos);
        lastpos = endpos;
        trajectory1 = drive.trajectoryBuilder(lastpos)
                .lineToLinearHeading(new Pose2d(-26, 15, Math.toRadians(0)) )
                .build();
    }
    public void AutoInit(HardwareMap hwm) {
        hardwareMap = hwm;
    }

    public void Runtra1(){
        drive.followTrajectory(trajectory1);
        lastpos = drive.getPoseEstimate();
    }
}



