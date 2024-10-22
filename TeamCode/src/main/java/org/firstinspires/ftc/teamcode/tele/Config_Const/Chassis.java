package org.firstinspires.ftc.teamcode.Tele.Config_Const;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.Tele.Config_Const.ConfigsNConst.LEFTFRONT;
import static org.firstinspires.ftc.teamcode.Tele.Config_Const.ConfigsNConst.LEFTREAR;
import static org.firstinspires.ftc.teamcode.Tele.Config_Const.ConfigsNConst.RIGHTFRONT;
import static org.firstinspires.ftc.teamcode.Tele.Config_Const.ConfigsNConst.RIGHTREAR;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.lib.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.drive.StandardTrackingWheelLocalizer;

public class Chassis {
    public double kp = 1;
    HardwareMap hardwareMap;
    SampleMecanumDrive drive;
    public Pose2d lastpos;
    public boolean isAuto = false;

    public Chassis(HardwareMap mp, Pose2d endpos)
    {
        this.hardwareMap = mp;
        this.drive = new SampleMecanumDrive(mp);
        this.drive.setPoseEstimate(endpos);
    }

    public void AutoInit(HardwareMap hwm) {
        hardwareMap=hwm;
    }

    public void TeleInit(HardwareMap hwm) {
        hardwareMap=hwm;
    }

    public void lowSpeed() {
        kp = 0.3;
    }

    public void normalSpeed() {
        kp = 1;
    }

    public void teleDrive(double x, double y, double rx)
    {
        if(!isAuto) {
            double leftFront = ((y - x - rx) * kp);
            double leftBack = ((y + x - rx) * kp);
            double rightFront = ((y + x + rx) * kp);
            double rightBack = ((y - x + rx) * kp);
            drive.setMotorPowers(leftFront, leftBack, rightFront, rightBack);
        }
    }

    public void gotopos(Pose2d endpose){
        lastpos = drive.getPoseEstimate();
        Trajectory trajectory = drive.trajectoryBuilder(lastpos)
                .lineToLinearHeading(endpose)
                .build();
        drive.followTrajectoryAsync(trajectory);
    }

    public void goauto(){
        isAuto = true;
    }

    public void gotele(){
        isAuto = false;
        if(drive.isBusy()){
            lastpos = drive.getPoseEstimate();
            Trajectory trajectory = drive.trajectoryBuilder(lastpos)
                    .lineToLinearHeading(lastpos)
                    .build();
            drive.followTrajectoryAsync(trajectory);
        }
    }

    public void autoUpdate(){
        if(isAuto) {
            drive.update();
        }
        if(!drive.isBusy()){
            isAuto = false;
        }
    }

    public void brake(){
        drive.setMotorPowers(0,0,0,0);
    }

    public void posvisual(Pose2d pos){
        if(pos.getHeading() != 0){
            Pose2d curpos = drive.getPoseEstimate();
            drive.setPoseEstimate(new Pose2d(pos.getX(), pos.getY(), curpos.getHeading()));
        }
    }

    public void correctHeading(double heading){
        Pose2d curpos = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(curpos.getX(), curpos.getY(), Math.toRadians(heading)));
    }

    public double returnHeading(){
        return Math.toDegrees(drive.getPoseEstimate().getHeading());
    }

}


