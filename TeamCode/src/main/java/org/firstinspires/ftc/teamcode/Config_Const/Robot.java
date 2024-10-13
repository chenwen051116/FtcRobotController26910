package org.firstinspires.ftc.teamcode.Config_Const;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;


public class Robot {
    public ElapsedTime timer = new ElapsedTime();
    public Arm cla = new Arm();
    public Chassis c = new Chassis();
    public boolean SecondDriver = false;

    public void AutoinitRemote(HardwareMap hwm){
        cla.autoInit(hwm);
        timer.reset();
        c.AutoInit(hwm);

    }

    public void TeleinitBlue(HardwareMap hwm){
        c.teleInit(hwm);
        cla.teleInit(hwm);
        timer.reset();
    }

    public void TeleinitRed(HardwareMap hwm){
        c.teleInit(hwm);
        cla.teleInit(hwm);
        timer.reset();
    }

    public void Teleinitshow(HardwareMap hwm){
        c.teleInit(hwm);
        cla.teleInit(hwm);
        timer.reset();
    }




    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
