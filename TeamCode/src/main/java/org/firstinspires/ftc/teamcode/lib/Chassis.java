package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.ext.roadrunner.drive.SampleMecanumDrive;

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
            double rightFront = ((y - x + rx) * kp);
            double rightBack = ((y + x + rx) * kp);
            double pmax = Math.max(Math.max(Math.max(leftFront, leftBack), rightFront), rightBack);
            if(pmax>1) {
                drive.setMotorPowers(leftFront/pmax, leftBack/pmax, rightFront/pmax, rightBack/pmax);
            }
            else {
                drive.setMotorPowers(leftFront, leftBack, rightFront, rightBack);
            }
        }
    }

    public void AutogotoPos(Pose2d endpose){
        lastpos = drive.getPoseEstimate();
        Trajectory trajectory = drive.trajectoryBuilder(lastpos)
                .lineToLinearHeading(endpose)
                .build();
        drive.followTrajectoryAsync(trajectory);
    }

    public void turnAutoMode(){
        isAuto = true;
    }

    public void turnTeleMode(){
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


