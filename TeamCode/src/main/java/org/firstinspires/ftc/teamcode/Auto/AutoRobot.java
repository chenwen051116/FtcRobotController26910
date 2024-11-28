package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class AutoRobot {
    public ElapsedTime timer = new ElapsedTime();
    public AutoArm cla = new AutoArm();
    //public boolean SecondDriver = false;
    HardwareMap hardwareMap;
    Pose2d endpose;
    public AutoChassis c = new AutoChassis(hardwareMap, endpose);

    public AutoRobot(HardwareMap mp, Pose2d endpos) {
        this.hardwareMap = mp;
        this.endpose = endpos;
    }

    public void AutoinitRemote(HardwareMap hwm) {
        cla.autoInit(hwm);
        timer.reset();
        c.AutoInit(hwm);

    }


    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
