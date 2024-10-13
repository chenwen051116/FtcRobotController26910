package org.firstinspires.ftc.teamcode.Config_Const;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.PIDController;
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
    PIDController mainpid = new PIDController(0.05,0.0,0.1);
    PIDController mainfpid = new PIDController(0.05,0.0,0.1);



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

    public void mainarmset(int pos){
        mainArml.setTargetPosition(pos);
        mainArml.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mainArml.setPower(0.3);
        mainArmr.setTargetPosition(pos);
        mainArmr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mainArmr.setPower(0.3);
    }

    public void drops() {
        claw.setPosition(0);
    }

    public void takes() {
        int k = 10;
        claw.setPosition(0);
        sleep(250);
        mainarmset(mainArml.getCurrentPosition()-k);
    }

    public void frontarmp(double power){
        if(mainArmf.getCurrentPosition()>10) {
            mainArmf.setPower(power);
        }
    }

    public void frontarmset(int pos){
        mainArmr.setTargetPosition(pos);
        mainArmr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mainArmr.setPower(0.3);
    }

    public void take(){
        flipArml.setPosition(0);
        flipArmr.setPosition(0);
        intakem.setPower(0.5);
    }

    public void trans(){
        flipArml.setPosition(0);
        flipArmr.setPosition(0);
        sleep(500);
        intakem.setPower(-0.2);
        sleep(500);
        intakem.setPower(0);
    }

    public void reve(){
        intakem.setPower(-0.2);
    }

    public void highbasket(){
        mainarmset(10);
    }

    public void lowbasket(){
        mainarmset(10);
    }

    public void highbar(){
        mainarmset(10);
    }

    public void lowbar(){
        mainarmset(10);
    }

    public void mainback(){
        mainarmset(0);
    }

    public void dump(){
        endArml.setPosition(0);
        endArmr.setPosition(0);
    }

    public void endback(){
        endArml.setPosition(0);
        endArmr.setPosition(0);
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
