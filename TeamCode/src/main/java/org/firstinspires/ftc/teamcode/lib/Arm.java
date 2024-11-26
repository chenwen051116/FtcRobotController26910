package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    public int armfpos = 5;



    public void autoInit(HardwareMap hwm) {

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
        mainArmf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakem.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakem.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        endback();
        trans();
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
        mainArmf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mainArmf.setPower(0.7);
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
        // 控制目标点
        armfpos += 10*power;
        if(armfpos > 1500) {
            mainArmf.setTargetPosition(-1500);
            armfpos = 1500;
        }
        else if (armfpos< 5) {
            mainArmf.setTargetPosition(-5);
            armfpos = 5;
        }
        else {
            mainArmf.setTargetPosition(-armfpos);
        }
        mainArmf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

//    public void frontarmset(int pos){
//        mainArmr.setPower(0.3);
//        mainArmr.setTargetPosition(pos);
//        mainArmr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//    }

    public void take(){//放下前面arm开始吸
        // 把两个 servo 放下去
        // 把滚吸过放下去
        flipArml.setPosition(0.8157);//左arm位置
        flipArmr.setPosition(0.2843);//右arm位置
        //gathering = true;
    }


    public void updatefm(){
        // rev 是吐出来
        // gathering 是正着吸
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
        // 滚吸收回来
        // gathering false 不再吸了
        flipArml.setPosition(0.1);//左arm位置
        flipArmr.setPosition(1);//右arm位置
        gathering = false;
    }

    public void fback(){//收回arm并反转
        // 快捷键收回横着的滑轨
        mainArmf.setTargetPosition(-5);
        armfpos = 5;
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

    public void lowbar(){
        mainarmset(1000);//低杆arm位置
    }

    public void spepos(){
        // 场边的 specimen 的位置
        mainarmset(147);//夹取样本位置
    }

    public void mainback(){
        //
        if(mainArmf.getCurrentPosition()>=-100 && mainArml.getCurrentPosition()>100) {
            mainArmf.setTargetPosition(-100);
            armfpos = 100;
            // set 到 100 避免冲突
        }

        mainarmset(5); // 竖着
    }

    public void dump(){//倒到框里
        double diffd=0.61-0.2767;
        endArml.setPosition(0.61-diffd);//左arm位置
        endArmr.setPosition(0.5+diffd);//右arm位置
    }

    public void endback(){//框里收回来
        double diffe=0.89-0.61;
        if(mainArmf.getCurrentPosition()>-100) {
            // 如果竖着的杆太低了，横着的杆又收的抬回来了。
            mainArmf.setTargetPosition(-100);
            armfpos = 100;
        }
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
