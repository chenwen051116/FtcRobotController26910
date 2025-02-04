package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "Testhus")
public class Testhus extends LinearOpMode {

    private HuskyLens HL;

    /**
     * This OpMode illustrates how to use the DFRobot HuskyLens.
     * <p>
     * The HuskyLens is a Vision Sensor with a built-in object detection model. It can
     * detect a number of predefined objects and AprilTags in the 36h11 family, can
     * recognize colors, and can be trained to detect custom objects. See this website for
     * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
     * <p>
     * This sample illustrates how to detect AprilTags, but can be used to detect other types
     * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
     * a name of "huskylens".
     */
    @Override
    public void runOpMode() {
        ElapsedTime myElapsedTime;

        HuskyLens.Arrow myHuskyLensBlock;

        HL = hardwareMap.get(HuskyLens.class, "HL");
        HuskyLens.Arrow[] myHuskyLensBlocks;
        // Put initialization blocks here.

       // telemetry.addData(">>", HL.? "Touch start to continue" : "Problem communicating with HuskyLens");
        HL.selectAlgorithm(HuskyLens.Algorithm.LINE_TRACKING);
        telemetry.update();
        myElapsedTime = new ElapsedTime();
        waitForStart();

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                if (myElapsedTime.seconds() >= 1) {
                    myElapsedTime.reset();
                    myHuskyLensBlocks = HL.arrows();
                    telemetry.addData("Block count", JavaUtil.listLength(myHuskyLensBlocks));
                    for (HuskyLens.Arrow myHuskyLensBlock_item : myHuskyLensBlocks) {
                        myHuskyLensBlock = myHuskyLensBlock_item;
                        telemetry.addData("Block", " size: " + myHuskyLensBlock.x_target+ "x" + myHuskyLensBlock.y_target + " position: " + myHuskyLensBlock.x_origin + "," + myHuskyLensBlock.y_origin);
                    }

                    telemetry.update();
                }
            }
        }
    }
}
