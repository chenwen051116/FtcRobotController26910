package org.firstinspires.ftc.teamcode.lib.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.lang.reflect.Array;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;


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
        limelight.pipelineSwitch(0);
        limelight.start();
        limelight.setPollRateHz(20);
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
   // private HuskyLens.Arrow[] myHuskyLensAs;
    private HuskyLens.Block[] myHuskyLensBlocks = null;
    public double hlGetAngle(HuskyLens.Block block) {
        if(block != null) {
            return VisionUtils.getStatus(block.width, block.height, block.x, block.y).getHeading();
        }
        else{
            return Math.PI;
        }
    }

    public BlockData getBlock(int color){
//        // HuskyLens implementation
//        myHuskyLensBlocks = HL.blocks();
//        telemetry.addData("Block count", JavaUtil.listLength(myHuskyLensBlocks));
//        for (HuskyLens.Block block : myHuskyLensBlocks) {
//            myHuskyLensBlock = block;
//            double ang = 114;
//            ang = VisionUtils.getStatus(block.width, block.height, block.x, block.y).getHeading();
//            if (Double.isNaN(ang)){
//                ang = 514;
//            }
//            telemetry.addData("Block", "id=" + myHuskyLensBlock.id + " size: " + myHuskyLensBlock.width + "x" + myHuskyLensBlock.height + " position: " + myHuskyLensBlock.x + "," + myHuskyLensBlock.y + " , angle: " + ang);
//            telemetry.update();
//            if(myHuskyLensBlock.id == id){
//                return myHuskyLensBlock;
//            }
//        }
//        telemetry.update();
        // LimeLight implementation
        limelight.start();
        limelight.pipelineSwitch(0);
        LLResult resultTmp = limelight.getLatestResult();
        telemetry.addData("the result is valid or not ", resultTmp.isValid());
        LLStatus status = limelight.getStatus();
        telemetry.addData("ll status", status.toString());
        double[] result = resultTmp.getPythonOutput();
        telemetry.addData("raw python output", "");
        for (double dbl : result){
            telemetry.addData("", dbl);
        }

        BlockData[] blocks = new BlockData[5];
        for(int i = 0; i < blocks.length; i++) {
            blocks[i] = new BlockData(result, 2 + i * 6);
            telemetry.addData("Block", "x=" + blocks[i].centerX + "; y=" + blocks[i].centerY + "; w=" + blocks[i].width + "; h=" + blocks[i].height + "; angle=" + blocks[i].angle + "; color=" + blocks[i].color);
            if((blocks[i].color & color) != 0) {
                telemetry.addData("Block", "Block detected at x=" + blocks[i].centerX + ", y=" + blocks[i].centerY);
                telemetry.update();
                return blocks[i];
            }
        }
        telemetry.update();
        limelight.close();
        return null;
    }

//    public HuskyLens.Arrow getA(){
//        myHuskyLensAs = HL.arrows();
//
//        return myHuskyLensAs[0];
//    }

    public double autoFocus(){
        BlockData block = getBlock(BlockData.COLOR_YELLOW | BlockData.COLOR_RED | BlockData.COLOR_BLUE);
        if(block != null && block.getAngle() < (Math.PI/4)){
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
