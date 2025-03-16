package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ext.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.lib.schedule.Scheduler;
import org.firstinspires.ftc.teamcode.lib.vision.Visual;


public class Robot {
    public ElapsedTime timer = new ElapsedTime();
    public Arm arm = new Arm();
    public Chassis chassis;
//    public Visual v = new Visual();
    //public boolean SecondDriver = false;
    HardwareMap hardwareMap;
    private Telemetry telemetry;

    public TrajectorySequence trajectorybar;
    public TrajectorySequence trajectorybarback;
    //public Visual v = new Visual();
    public Robot(HardwareMap mp, Scheduler scheduler) {
        this.hardwareMap = mp;
        chassis = new Chassis(hardwareMap);
        this.arm.setScheduler(scheduler);
    }

    public void autoInit(HardwareMap hwm) {
        arm.autoInit(hwm);
        timer.reset();
        chassis.AutoInit(hwm);

    }

    public void teleInit(HardwareMap hwm, Telemetry telemetry) {
        chassis.TeleInit(hwm);
        arm.teleInit(hwm);
        timer.reset();
        this.telemetry = telemetry;
//        v.teleInit(hwm, telemetry);

        trajectorybar = chassis.drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    arm.highBar();//把arm伸上去
                    chassis.turnAutoMode();
                    //arm.speClaw.setPosition(0.15);
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(28.8,31.72,3.14))
                .build();

        trajectorybarback = chassis.drive.trajectorySequenceBuilder(new Pose2d(28.8,31.72,3.14))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    chassis.turnAutoMode();
                })
                .lineToLinearHeading(new Pose2d(0,0,0))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    arm.takeSpePos();//把arm伸上去
                    //arm.speClaw.setPosition(0.15);
                })

                .build();
        //v.apt();
    }

    public void barAuto(){
        chassis.drive.followTrajectorySequenceAsync(trajectorybar);
        chassis.turnTeleMode();
    }

    public void barbackAuto(){
        chassis.drive.followTrajectorySequenceAsync(trajectorybarback);
        chassis.turnTeleMode();
    }


    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

//    public void updatevisualposMT1(){
//        c.posvisual(v.getfieldposeMT1());
//    }
//
//    public void updatevisualposMT2(){
//        c.posvisual(v.getfieldposeMT2());
//    }

}
