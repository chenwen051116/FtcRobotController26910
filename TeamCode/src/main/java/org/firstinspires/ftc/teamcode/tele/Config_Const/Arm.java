package org.firstinspires.ftc.teamcode.Tele.Config_Const;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Timer;

public class Arm {
    public DcMotorEx mainArml = null;
    public DcMotorEx mainArmr = null;
    public DcMotorEx mainArmf = null;
    public DcMotorEx intakem = null;
    public Servo claw = null;
    public Servo flipArml = null;
    public Servo flipArmr = null;
    public Servo endArml = null;
    public Servo endArmr = null;
    public boolean rev = false;
    public boolean gathering = false;



    public void autoInit(HardwareMap hwm) {
        mainArml = hwm.get(DcMotorEx.class, "al");
        mainArmr = hwm.get(DcMotorEx.class, "ar");
        mainArmf = hwm.get(DcMotorEx.class, "af");
        intakem = hwm.get(DcMotorEx.class, "in");
        claw = hwm.get(Servo.class, "cl");
        flipArml = hwm.get(Servo.class, "fal");
        flipArmr = hwm.get(Servo.class, "far");
        endArml = hwm.get(Servo.class, "eal");
        endArmr = hwm.get(Servo.class, "eal");
        mainArml.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mainArml.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainArml.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mainArmr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mainArmr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainArmr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mainArmf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mainArmf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainArmf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakem.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakem.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        takes();
    }

    public void teleInit(HardwareMap hwm) {
        intakem = hwm.get(DcMotorEx.class, "in");
        mainArml = hwm.get(DcMotorEx.class, "al");
        mainArmr = hwm.get(DcMotorEx.class, "ar");
        mainArmf = hwm.get(DcMotorEx.class, "af");
        claw = hwm.get(Servo.class, "cL");
        flipArml = hwm.get(Servo.class, "lia");
        flipArmr = hwm.get(Servo.class, "ria");
        endArml = hwm.get(Servo.class, "oL");
        endArmr = hwm.get(Servo.class, "oR");
        mainArml.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mainArml.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainArml.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mainArmr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mainArmr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainArmr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mainArmf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mainArmr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainArmf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakem.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakem.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        endback();
        trans();
        claw.setPosition(0.37);
    }

    public void mainarmset(int pos){
        mainArml.setTargetPosition(pos);
        mainArml.setPower(0.7);
        mainArml.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mainArmr.setPower(0.7);
        mainArmr.setTargetPosition(-pos);
        mainArmr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void takes() {//夹样本
        //爪子夹样本的位置
        claw.setPosition(0.15);
    }

    public void drops() {//放下样本松手
        int k = 350;    //向下移动多少
        if(mainArml.getCurrentPosition()>1000) {
            mainarmset(mainArml.getCurrentPosition() - k);
            sleep(500);
        }

        claw.setPosition(0.37);//松手舵机位置
        
    }
//mudheadcar
    public void frontarmp(double power){
        if(power>0 && mainArmf.getCurrentPosition() > 1500) {
            mainArmf.setPower(0);
        }
        else {
            mainArmf.setPower(power);
        }
    }

    public void frontarmset(int pos){
        mainArmr.setPower(0.3);
        mainArmr.setTargetPosition(pos);
        mainArmr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void take(){//放下前面arm开始吸
        flipArml.setPosition(0.8157);//左arm位置
        flipArmr.setPosition(0.2843);//右arm位置
        //gathering = true;
    }


    public void updatefm(){
        if(rev) {
            intakem.setPower(1);//滚吸反转功率
        }
        else if(gathering) {
            intakem.setPower(-1);//滚吸功率
        }
        else{
            intakem.setPower(0);
        }
    }

    public void trans(){//收回arm并反转
        flipArml.setPosition(0.1);//左arm位置
        flipArmr.setPosition(1);//右arm位置
        gathering = false;
    }

    public void reve(){
        rev = true;
    }

    public void rollback(){
        rev = false;
    }

    public void highbasket(){
        mainarmset(2755);//高框arm位置
    }

    public void lowbasket(){
        mainarmset(1242);//低框arm位置
    }

    public void highbar(){
        mainarmset(1561);//高杆arm位置
    }

    //public void lowbar(){
    //    mainarmset(10);//低杆arm位置
    //}

    public void spepos(){
        mainarmset(147);//夹取样本位置
    }

    public void mainback(){
        mainarmset(10);
    }

    public void dump(){//倒到框里
        double diffd=0.61-0.2767;
        endArml.setPosition(0.61-diffd);//左arm位置
        endArmr.setPosition(0.5+diffd);//右arm位置
    }

    public void endback(){//框里收回来
        double diffe=0.89-0.61;
        endArml.setPosition(0.61+diffe);//左arm位置
        endArmr.setPosition(0.5-diffe);//右arm位置
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
