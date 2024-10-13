package org.firstinspires.ftc.teamcode.Config_Const;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.Config_Const.ConfigsNConst.LEFTFRONT;
import static org.firstinspires.ftc.teamcode.Config_Const.ConfigsNConst.LEFTREAR;
import static org.firstinspires.ftc.teamcode.Config_Const.ConfigsNConst.RIGHTFRONT;
import static org.firstinspires.ftc.teamcode.Config_Const.ConfigsNConst.RIGHTREAR;

public class Chassis {
    public DcMotorEx leftFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightBack = null;
    public double kp = 1;

    public void AutoInit(HardwareMap hwm) {
        leftFront = hwm.get(DcMotorEx.class, LEFTFRONT);
        rightFront = hwm.get(DcMotorEx.class, RIGHTFRONT);
        leftBack = hwm.get(DcMotorEx.class, LEFTREAR);
        rightBack = hwm.get(DcMotorEx.class, RIGHTREAR);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }public void teleInit(HardwareMap hwm) {
        leftFront = hwm.get(DcMotorEx.class, LEFTFRONT);
        rightFront = hwm.get(DcMotorEx.class, RIGHTFRONT);
        leftBack = hwm.get(DcMotorEx.class, LEFTREAR);
        rightBack = hwm.get(DcMotorEx.class, RIGHTREAR);

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    public void lowSpeed() {
        kp = 0.3;
    }

    public void normalSpeed() {
        kp = 1;
    }
    public void teleDrive(double x, double y, double rx) {
        leftFront.setPower((y - x - rx) * kp);
        leftBack.setPower((y + x - rx) * kp);
        rightFront.setPower((y + x + rx) * kp);
        rightBack.setPower((y - x + rx) * kp);
    }

    public void brake(){
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

}


