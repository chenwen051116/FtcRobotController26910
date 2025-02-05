package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class Visual {
    private Limelight3A limelight;
    private IMU imu;
    private Telemetry telemetry;

    public void teleInit(HardwareMap hwm, Telemetry telemetry) {
        this.telemetry = telemetry;
        HL = hwm.get(HuskyLens.class, "HL");
        HuskyLens.Block[] myHuskyLensBlocks = HL.blocks();
        HL.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

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


    public void apt() {
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public Pose2d getfieldposeMT1() {
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
        return new Pose2d(x, y, d);
    }

    public void updateyaw(double yaw) {
        limelight.updateRobotOrientation(yaw);
    }

    public void selfyaw(double yaw) {
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
    }

    public Pose2d getfieldposeMT2() {
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
        return new Pose2d(x, y, d);
    }

    public void llstop() {
        limelight.stop();
    }


    private HuskyLens HL;
    private HuskyLens.Block myHuskyLensBlock;
   // private HuskyLens.Arrow[] myHuskyLensAs;
    private HuskyLens.Block[] myHuskyLensBlocks = null;
    public double hlGetAngle(HuskyLens.Block block) {
        if(block != null) {
            return VisionUtils.getStatus(block.width, block.height, block.x, block.y).getHeading();
        }
        else{
            return 3.14;
        }
    }

    public HuskyLens.Block getBlock(int id){
        myHuskyLensBlocks = HL.blocks();
        telemetry.addData("Block count", JavaUtil.listLength(myHuskyLensBlocks));
        for (HuskyLens.Block block : myHuskyLensBlocks) {
            myHuskyLensBlock = block;
            double ang = 114;
            ang = VisionUtils.getStatus(block.width, block.height, block.x, block.y).getHeading();
            if (Double.isNaN(ang)){
                ang = 514;
            }
            telemetry.addData("Block", "id=" + myHuskyLensBlock.id + " size: " + myHuskyLensBlock.width + "x" + myHuskyLensBlock.height + " position: " + myHuskyLensBlock.x + "," + myHuskyLensBlock.y + " , angle: " + ang);
            telemetry.update();
            if(myHuskyLensBlock.id == id){
                return myHuskyLensBlock;
            }
        }
        telemetry.update();
        return null;
    }

    public HuskyLens.Block getBlockNear(){
        myHuskyLensBlocks = HL.blocks();
        for (HuskyLens.Block myHuskyLensBlock_item : myHuskyLensBlocks) {
            myHuskyLensBlock = myHuskyLensBlock_item;

                return myHuskyLensBlock;
            //telemetry.addData("Block", "id=" + myHuskyLensBlock.id + " size: " + myHuskyLensBlock.width + "x" + myHuskyLensBlock.height + " position: " + myHuskyLensBlock.x + "," + myHuskyLensBlock.y);
        }
        return null; //telemetry.addData("Block", "id=" + myHuskyLensBlock.id + " size: " + myHuskyLensBlock.width + "x" + myHuskyLensBlock.height + " position: " + myHuskyLensBlock.x + "," + myHuskyLensBlock.y);
    }

//    public HuskyLens.Arrow getA(){
//        myHuskyLensAs = HL.arrows();
//
//        return myHuskyLensAs[0];
//    }

    public double autoFocus(){
        if(hlGetAngle(getBlock(1)) < (3.14/4)){
            return 0.5;
        }
        else{
            return 0;
        }
    }

//    public double autoFocus(){
//        return Vu.getServoValFromArrow(getA().x_origin,getA().y_origin,getA().x_target,getA().y_target);
//    }




}
