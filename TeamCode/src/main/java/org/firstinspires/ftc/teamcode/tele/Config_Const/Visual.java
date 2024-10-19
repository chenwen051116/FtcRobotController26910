package org.firstinspires.ftc.teamcode.Tele.Config_Const;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class Visual {
    private Limelight3A limelight;
    private IMU imu;

    public void teleInit(HardwareMap hwm) {
        imu = hwm.get(IMU.class, "imu");
        limelight = hwm.get(Limelight3A.class, "lm");
        limelight.setPollRateHz(100);
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );
    }

    public void apt(){
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public Pose2d getfieldposeMT1(){
        double x = 0;
        double y = 0;
        double d = 0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    x = botpose.getPosition().x * 39.3701;
                    y = botpose.getPosition().y * 39.3701;
                    d = 1;
                }
            }
        }
        return new Pose2d(x,y,d);
    }

    public void updateyaw(double yaw){
        limelight.updateRobotOrientation(yaw);
    }

    public void selfyaw(double yaw){
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
    }

    public Pose2d getfieldposeMT2(){
        double x = 0;
        double y = 0;
        double d = 0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2();
                if (botpose != null) {
                    x = botpose.getPosition().x * 39.3701;
                    y = botpose.getPosition().y * 39.3701;
                    d = 1;
                }
            }
        }
        return new Pose2d(x,y,d);
    }

    public void llstop(){
        limelight.stop();
    }

}
