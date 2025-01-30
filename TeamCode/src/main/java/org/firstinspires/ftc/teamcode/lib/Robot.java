package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.schedule.Scheduler;


public class Robot {
    public ElapsedTime timer = new ElapsedTime();
    public Arm arm = new Arm();
    public Chassis chassis;
    public Visual v = new Visual();
    //public boolean SecondDriver = false;
    HardwareMap hardwareMap;

    //public Visual v = new Visual();
    public Robot(HardwareMap mp, Scheduler scheduler) {
        this.hardwareMap = mp;
        chassis = new Chassis(hardwareMap);
        this.arm.setScheduler(scheduler);
    }

    public void Autoinit(HardwareMap hwm) {
        arm.autoInit(hwm);
        timer.reset();
        chassis.AutoInit(hwm);

    }

    public void Teleinit(HardwareMap hwm) {
        chassis.TeleInit(hwm);
        arm.teleInit(hwm);
        timer.reset();
        v.teleInit(hwm);
        //v.apt();
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
