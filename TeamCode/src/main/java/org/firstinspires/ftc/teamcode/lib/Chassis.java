package org.firstinspires.ftc.teamcode.lib;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ext.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ext.roadrunner.trajectorysequence.TrajectorySequence;

public class Chassis {
    public double kp = 1;
    public Pose2d lastpos;
    public boolean isAuto = false;

    public Pose2d endPos = new Pose2d(0, 0, Math.toRadians(0));
    HardwareMap hardwareMap;
    SampleMecanumDrive drive;

    TrajectorySequence trajectoryse;

    public Chassis(HardwareMap mp) {
        this.hardwareMap = mp;
        this.drive = new SampleMecanumDrive(mp);
    }

    public void AutoInit(HardwareMap hwm) {
        hardwareMap = hwm;
    }

    public void TeleInit(HardwareMap hwm) {
        hardwareMap = hwm;
        this.drive.setPoseEstimate(endPos);
    }

    public void lowSpeed() {
        kp = 0.3;
    }

    public void normalSpeed() {
        kp = 1;
    }

    public void teleDrive(double x, double y, double rx) {
        if (!isAuto) {
            double leftFront = ((y - x - rx) * kp);
            double leftBack = ((y + x - rx) * kp);
            double rightFront = ((y - x + rx) * kp);
            double rightBack = ((y + x + rx) * kp);
            double pmax = Math.max(Math.max(Math.max(abs(leftFront), abs(leftBack)), abs(rightFront)), abs(rightBack));
            if (pmax > 1) {
                drive.setMotorPowers(-leftFront / pmax, -leftBack / pmax, -rightFront / pmax, -rightBack / pmax);
            } else {
                drive.setMotorPowers(-leftFront, -leftBack, -rightFront, -rightBack);
            }
        }
    }

    public void AutogotoPos(Pose2d endpose) {
        lastpos = drive.getPoseEstimate();
        Trajectory trajectory = drive.trajectoryBuilder(lastpos)
                .lineToLinearHeading(endpose)
                .build();
        drive.followTrajectoryAsync(trajectory);
    }


    public void setOrigin() {
        drive.setPoseEstimate(new Pose2d(0,0,0));
    }
    public void goOriginPath() {
        lastpos = drive.getPoseEstimate();
        trajectoryse = drive.trajectorySequenceBuilder(lastpos)
                .lineToLinearHeading(endPos)
                .build();
    }

    public void goOrigin(){
        turnAutoMode();
        drive.followTrajectorySequence(trajectoryse);

    }

    public void cancelAuto(){
        if(!drive.isBusy()){
            isAuto = false;
        }
    }

    public void turnAutoMode() {
        isAuto = true;
    }

    public void turnTeleMode() {
        isAuto = false;
        lastpos = drive.getPoseEstimate();
        if (drive.isBusy()) {

            lastpos = drive.getPoseEstimate();
            Trajectory trajectory = drive.trajectoryBuilder(lastpos)
                    .lineToLinearHeading(lastpos)
                    .build();
            drive.followTrajectory(trajectory);
        }
    }

    public void autoUpdate() {
        if (isAuto) {
            drive.update();
        }
        if (!drive.isBusy()) {
            isAuto = false;
        }
    }

    public void brake() {
        drive.setMotorPowers(0, 0, 0, 0);
    }

    public void posvisual(Pose2d pos) {
        if (pos.getHeading() != 0) {
            Pose2d curpos = drive.getPoseEstimate();
            drive.setPoseEstimate(new Pose2d(pos.getX(), pos.getY(), curpos.getHeading()));
        }
    }

    public void correctHeading(double heading) {
        Pose2d curpos = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(curpos.getX(), curpos.getY(), Math.toRadians(heading)));
    }

    public double returnHeading() {
        return Math.toDegrees(drive.getPoseEstimate().getHeading());
    }

}


