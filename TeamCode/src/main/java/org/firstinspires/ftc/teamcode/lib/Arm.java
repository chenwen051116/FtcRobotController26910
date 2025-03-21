package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ext.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.lib.schedule.Scheduler;

public class Arm {
    public DcMotorEx VtLeft = null;
    public DcMotorEx VtRight = null;
    public DcMotorEx hzFront = null;
    public Servo speClaw = null;
    public Servo inArmLeft = null;
    public Servo inArmRight = null;
    public Servo inAngleLeft = null;
    public Servo inAngleRight = null;
    public Servo inAngleTurn = null;
    public Servo inClaw = null;
    public Servo outArmLeft = null;
    public Servo outArmRight = null;
    public int frontArmPos = 0;
    public boolean backPos = true;
    public boolean hori = false;
    public boolean delayhori = false;
    public Scheduler scheduler;



    public void autoInit(HardwareMap hwm) {
    }

    public void teleInit(HardwareMap hwm) {
        VtLeft = hwm.get(DcMotorEx.class, "vtSlider_lfMt");
        VtRight = hwm.get(DcMotorEx.class, "vtSlider_rtMt");
        hzFront = hwm.get(DcMotorEx.class, "hzSlider_mt");
        speClaw = hwm.get(Servo.class, "bkSpClaw_sv");
        inArmLeft = hwm.get(Servo.class, "frSpFlipper_lfSv");
        inArmRight = hwm.get(Servo.class, "frSpFlipper_rtSv");
        inAngleLeft = hwm.get(Servo.class, "frTurn_lfSv");
        inAngleRight = hwm.get(Servo.class, "frTurn_rtSv");
        inAngleTurn = hwm.get(Servo.class, "frAiming_Sv");
        inClaw = hwm.get(Servo.class, "frInClaw_Sv");
        outArmLeft = hwm.get(Servo.class, "bkSpFlipper_lfSv");
        outArmRight = hwm.get(Servo.class, "bkSpFlipper_rtSv");
        VtLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //VtLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VtLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        VtRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //VtRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VtRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hzFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //hzFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hzFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        inClaw.setPosition(0.4);
        inArmLeft.setPosition(0.5);//左arm位置
        inArmRight.setPosition(0.65);//右arm位置
        backPos = true;
        TurnSet(0.1);
        basketBack();



        speClaw.setPosition(0.37);
        HzArmSet(5);
        inTurn(0);


    }


    public void VtArmSet(int pos) {
        if(pos < 100) {
            VtLeft.setPower(0.7);
            VtRight.setPower(0.7);
        }
        else{
            VtLeft.setPower(1);
            VtRight.setPower(1);
        }
        VtLeft.setTargetPosition(pos);
        VtLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VtRight.setTargetPosition(-pos);
        VtRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void TurnSet(double pos){
        inAngleLeft.setPosition(pos);
       inAngleRight.setPosition(0+pos);
    }

    public void ArmSet(double pos){
        inArmLeft.setPosition(0.5+pos);
        inArmRight.setPosition(0.65-pos);
    }

    public void HzArmSet(int pos) {
        hzFront.setPower(0.8);
        frontArmPos = pos;
        hzFront.setTargetPosition(-frontArmPos);
        hzFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void closeClaw() {//夹样本
        //爪子夹样本的位置
        speClaw.setPosition(0.15);
    }

    public void dropSpe() {//放下样本松手
        int k = 500;    //向下移动多少
        if(VtLeft.getCurrentPosition() > 800) {
            VtArmSet(VtLeft.getCurrentPosition() - k);
            scheduler.addTaskAfter(300, new Runnable() {
                @Override
                public void run() {
                    speClaw.setPosition(0.37);//松手舵机位置
                }
            });
        } else {
            speClaw.setPosition(0.37);//松手舵机位置
        }


    }

    //mudheadcar
    public void HzArmVel(double power) {
        // 控制目标点
        frontArmPos += 10 * power;
        if (frontArmPos > 1450) {
            HzArmSet(1450);
        } else if (frontArmPos < 0) {
            HzArmSet(0);
        } else {
            HzArmSet(frontArmPos);
        }

    }

    public void HzArmVellim(double power) {
        // 控制目标点
        frontArmPos += 10 * power;
        if (frontArmPos > 1050) {
            HzArmSet(1050);
        } else if (frontArmPos < 0) {
            HzArmSet(0);
        } else {
            HzArmSet(frontArmPos);
        }

    }

    public void frontIntakeDownquick() {//放下前面arm开始吸
        // 把两个 servo 放下去
        // 把滚吸过放下去
        if(!backPos) {
            TurnSet(0);
            inClaw.setPosition(0.4);
                    inClaw.setPosition(0.4);

            scheduler.addTaskAfter(100, new Runnable() {
                @Override
                public void run() {
                    ArmSet(0.31);
                }
            });
            scheduler.addTaskAfter(200, new Runnable() {
                @Override
                public void run() {
                    inClaw.setPosition(0.8);
                }
            });
            scheduler.addTaskAfter(500, new Runnable() {
                @Override
                public void run() {
                    ArmSet(0.19);
                    TurnSet(0);

                }
            });
            scheduler.addTaskAfter(1000, new Runnable() {
                @Override
                public void run() {
                    backPos = false;
                }
            });
        } else {
            ArmSet(0.19);
            TurnSet(0);
            scheduler.addTaskAfter(300, new Runnable() {
                @Override
                public void run() {
                    backPos = false;
                }
            });
        }
    }

    public void vtArmlowreset(){
        VtLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
       // VtRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //VtRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VtRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        VtLeft.setPower(-0.2);
        VtRight.setPower(0.2);
    }

    public void vtArmreset(){
        VtLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // VtRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //VtRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VtRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        VtLeft.setPower(0);
        VtRight.setPower(0);
        VtLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        VtRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void frontIntakeDown() {//放下前面arm开始吸
        // 把两个 servo 放下去
        // 把滚吸过放下去
        if(!backPos) {
            TurnSet(0);
            inClaw.setPosition(0.4);
            scheduler.addTaskAfter(100, new Runnable() {
                @Override
                public void run() {
                    inClaw.setPosition(0.4);
                    ArmSet(0.31);
                }
            });

            scheduler.addTaskAfter(400, new Runnable() {
                @Override
                public void run() {
                    inClaw.setPosition(0.8);
                }
            });
            scheduler.addTaskAfter(600, new Runnable() {
                @Override
                public void run() {
                    ArmSet(0.15);
                    TurnSet(0.32);

                }
            });
            scheduler.addTaskAfter(1000, new Runnable() {
                @Override
                public void run() {
                    backPos = false;
                }
            });
        } else {
            ArmSet(0.15);
            TurnSet(0.32);
            scheduler.addTaskAfter(300, new Runnable() {
                @Override
                public void run() {
                    backPos = false;
                }
            });
        }
    }

    public void frontIntakeDownSlow() {//放下前面arm开始吸
        // 把两个 servo 放下去
        // 把滚吸过放下去
        if(!backPos) {
            TurnSet(0);
            inClaw.setPosition(0.4);
            scheduler.addTaskAfter(300, new Runnable() {
                @Override
                public void run() {
                    inClaw.setPosition(0.4);
                    ArmSet(0.31);
                }
            });

            scheduler.addTaskAfter(600, new Runnable() {
                @Override
                public void run() {
                    inClaw.setPosition(0.8);
                }
            });
            scheduler.addTaskAfter(1000, new Runnable() {
                @Override
                public void run() {
                    ArmSet(0.15);
                    TurnSet(0.32);

                }
            });
            scheduler.addTaskAfter(1400, new Runnable() {
                @Override
                public void run() {
                    backPos = false;
                }
            });
        } else {
            ArmSet(0.15);
            TurnSet(0.32);
            scheduler.addTaskAfter(300, new Runnable() {
                @Override
                public void run() {
                    backPos = false;
                }
            });
        }
    }

    public void inTurn(double num){
        inAngleTurn.setPosition(0.5+num);
    }

    public void switchinTurn(){
        if(!delayhori) {
            delayhori = true;
            if (hori) {
                inTurn(0);
            } else {
                inTurn(0.5);
            }

            scheduler.addTaskAfter(500, new Runnable() {
                @Override
                public void run() {
                    delayhori = false;
                    if (hori) {
                        hori = false;
                    } else {
                        hori = true;
                    }
                }
            });

        }
    }
    public void releaseC(){
        inClaw.setPosition(0.3);
    }

    public void inArmTrans() {//收回arm并反转
        // 滚吸收回来
        // getIntake false 不再吸了
        HzArmSet(5);
        if(-hzFront.getCurrentPosition()<10) {
            ArmSet(-0.15);
            TurnSet(1);
//            inTurn (0);
            if(inAngleTurn.getPosition()>0.8){
                inTurn(0.5);
            }
            else{
                inTurn(-0.5);
            }



           scheduler.addTaskAfter(600, new Runnable() {
                @Override
                public void run() {
                    inClaw.setPosition(0.3);
                    //inTurn(0.5);
                    hori = true;
                    //backPos = true;
                }
            });
            scheduler.addTaskAfter(1000, new Runnable() {
                @Override
                public void run() {
                    ArmSet(-0.1);

                    //backPos = true;
                }
            });
            scheduler.addTaskAfter(1400, new Runnable() {
                @Override
                public void run() {
                    ArmSet(0.19);
                    TurnSet(0);
                    //backPos = true;
                }
            });
        }
    }



    public void frontArmBack() {//收回arm并反转
        // 快捷键收回横着的滑轨
        HzArmSet(5);
    }


    public void highBasket() { VtArmSet(2800);}

    public void lowBasket() {
        VtArmSet(1242);//低框arm位置
    }

    public void highBar() {
        VtArmSet(1250);//高杆arm位置
    }

    public void lowBar() {
        VtArmSet(1000);//低杆arm位置
    }

    public void takeSpePos() {
        // 场边的 specimen 的位置
        if(frontArmPos <= 100  && VtLeft.getCurrentPosition() > 200) {
            HzArmSet(200);
            // set 到 200 避免冲突
        }
        VtArmSet(147);//夹取样本位置

    }

    public void VtBack() {
        //
        if (frontArmPos <= 100 && VtLeft.getCurrentPosition() > 200) {
            HzArmSet(200);

            // set 到 200 避免冲突
        }

        VtArmSet(0); // 竖着
        if (VtLeft.getCurrentPosition() < 5){
            VtLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            VtRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    }

    public void basketOut() {//倒到框里
        double d= 0.37;
        outArmLeft.setPosition(0.61 + d);//左arm位置
        outArmRight.setPosition(0.5 - d);//右arm位置

    }

    public void finalClimb() {//倒到框里
        double d= 0.3;
        outArmLeft.setPosition(0.61 + d);//左arm位置
        outArmRight.setPosition(0.5 - d);//右arm位置
        speClaw.setPosition(0.6);//松手舵机位置
        highBasket();
        HzArmSet(0);

    }
    public void basketBack() {//框里收回来
        double d = 0.4233;
        outArmLeft.setPosition(0.61 - d);//左arm位置
        outArmRight.setPosition(0.5 + d);//右arm位置
        if(frontArmPos < 200 && VtLeft.getCurrentPosition() < 200) {
            // 如果竖着的杆太低了，横着的杆又收的抬回来了。
            HzArmSet(200);
        }
    }

    public void setScheduler(Scheduler scheduler) {
        this.scheduler = scheduler;
    }

    private void sleep(long ms){
        try{
            Thread.sleep(ms);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }
}
