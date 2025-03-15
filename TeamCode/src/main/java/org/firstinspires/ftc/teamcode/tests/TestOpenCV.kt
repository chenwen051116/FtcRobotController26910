package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.hardware.dfrobot.HuskyLens
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.opencv.core.*

@TeleOp(name = "TestOpenCV")
class TestOpenCV : LinearOpMode() {
    /**
     * This OpMode illustrates how to use the DFRobot HuskyLens.
     *
     *
     * The HuskyLens is a Vision Sensor with a built-in object detection model. It can
     * detect a number of predefined objects and AprilTags in the 36h11 family, can
     * recognize colors, and can be trained to detect custom objects. See this website for
     * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
     *
     *
     * This sample illustrates how to detect AprilTags, but can be used to detect other types
     * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
     * a name of "huskylens".
     */

    override fun runOpMode() {
        // create a 10x10 mat random mat
        waitForStart()
        if (opModeIsActive()){
            while(opModeIsActive()) {
                val mat = Mat(10, 10, CvType.CV_8UC3)
                // randomize
                Core.randu(mat, 0.0, 255.0)
                // print the mat
                telemetry.addData("Mat", mat.dump())
                telemetry.update()
            }
        }
    }
}