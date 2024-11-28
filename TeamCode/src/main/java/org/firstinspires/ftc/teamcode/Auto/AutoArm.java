package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoArm {
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
        mainArml = hwm.get(DcMotorEx.class, "ml");
        mainArmr = hwm.get(DcMotorEx.class, "mr");
        mainArmf = hwm.get(DcMotorEx.class, "mf");
        intakem = hwm.get(DcMotorEx.class, "im");
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
        mainArml = hwm.get(DcMotorEx.class, "ml");
        mainArmr = hwm.get(DcMotorEx.class, "mr");
        mainArmf = hwm.get(DcMotorEx.class, "mf");
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
        mainArmf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakem.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakem.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void mainarmset(int pos) {
        mainArml.setTargetPosition(pos);
        mainArml.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mainArml.setPower(0.3);
        mainArmr.setTargetPosition(pos);
        mainArmr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mainArmr.setPower(0.3);
    }

    public void takes() {//夹样本
        //爪子夹样本的位置
        claw.setPosition(0);
    }

    public void drops() {//放下样本松手
        int k = 10;    //向下移动多少
        mainarmset(mainArml.getCurrentPosition() - k);
        sleep(250);
        claw.setPosition(0);//松手舵机位置

    }

    public void frontarmp(double power) {
        if (mainArmf.getCurrentPosition() > 10) {
            mainArmf.setPower(power);
        }
    }

    public void frontarmset(int pos) {
        mainArmr.setTargetPosition(pos);
        mainArmr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mainArmr.setPower(0.3);
    }

    public void take() {//放下前面arm开始吸
        flipArml.setPosition(0);//左arm位置
        flipArmr.setPosition(0);//右arm位置
        gathering = true;
    }

    public void updatefm() {
        if (rev) {
            intakem.setPower(-0.2);//滚吸反转功率
        } else if (gathering) {
            intakem.setPower(0.5);//滚吸功率
        } else {
            intakem.setPower(0);
        }
    }

    public void trans() {//收回arm并反转
        flipArml.setPosition(0);//左arm位置
        flipArmr.setPosition(0);//右arm位置
        gathering = false;
    }

    public void reve() {
        rev = true;
    }

    public void rollback() {
        rev = false;
    }

    public void highbasket() {
        mainarmset(10);//高框arm位置
    }

    public void lowbasket() {
        mainarmset(10);//低框arm位置
    }

    public void highbar() {
        mainarmset(10);//高杆arm位置
    }

    public void lowbar() {
        mainarmset(10);//低杆arm位置
    }

    public void spepos() {
        mainarmset(10);//夹取样本位置
    }

    public void mainback() {
        mainarmset(0);
    }

    public void dump() {//倒到框里
        endArml.setPosition(0);//左arm位置
        endArmr.setPosition(0);//右arm位置
    }

    public void endback() {//框里收回来
        endArml.setPosition(0);//左arm位置
        endArmr.setPosition(0);//右arm位置
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
