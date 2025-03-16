package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.webcam_visual.common.Block;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import kotlin.Pair;


public class AutoRobot {
    public final List<Block> blockList = new ArrayList<>();
    public void getBlocks() {
        // TODO: To be implemented with computer vision
    }
    public Thread startVisionThread() {
        // TODO: To be implemented with computer vision
    }
    public ElapsedTime timer = new ElapsedTime();
    public AutoArm arm = new AutoArm();
    public AutoChassis chassis;
    //public boolean SecondDriver = false;
    HardwareMap hardwareMap;
    Pose2d endPos;

    //public Visual v = new Visual();
    public AutoRobot(HardwareMap mp) {
        this.hardwareMap = mp;
        chassis = new AutoChassis(hardwareMap);
    }

    public void Autoinit(HardwareMap hwm) {
        arm.autoInit(hwm);
        timer.reset();
        chassis.AutoInit(hwm);
        arm.closeClaw();
    }

    public void Teleinit(HardwareMap hwm) {
        chassis.TeleInit(hwm);
        arm.teleInit(hwm);
        timer.reset();
        //v.apt();
    }

    public void trackBlock(Block block) {
        if(block == null) {
            return;
        }
        float unitToArmMovement = 100.0f;
        float unitToWheelMovement = 0.8f;
        int armCurrentPos = -arm.hzFront.getCurrentPosition();
        int nextPosition = (int) (block.getCenter().getSecond() * unitToArmMovement) + armCurrentPos;
        arm.HzArmSet(Math.max(0, Math.min(1450, nextPosition)));
        double motorPower = block.getCenter().getFirst() * unitToWheelMovement;
        chassis.drive.setMotorPowers(motorPower, -motorPower, -motorPower, motorPower);
        sleep(200);
        chassis.drive.setMotorPowers(0, 0, 0, 0);
    }

    public static Block getBlockAtCenter(List<Block> blocks) {
        Block ans = null;
        for(Block block : blocks) {
            if(ans == null) {
                ans = block;
            } else {
                Pair<Float, Float> center = block.getCenter();
                Pair<Float, Float> ansCenter = ans.getCenter();
                if(center.getFirst() * center.getFirst() + center.getSecond() * center.getSecond()
                 < ansCenter.getFirst() * ansCenter.getFirst() + ansCenter.getSecond() * ansCenter.getSecond()) {
                    ans = block;
                }
            }
        }
        return ans;
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
