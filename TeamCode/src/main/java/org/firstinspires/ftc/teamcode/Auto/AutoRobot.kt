package org.firstinspires.ftc.teamcode.Auto

import CameraHelper
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.webcam_visual.common.Block
import org.webcam_visual.common.COLOR_DEF_R9000P
import org.webcam_visual.detectors.ColorBlockDetector
import org.webcam_visual.pipeline.RobotVisionPipeline
import org.webcam_visual.preproc.BilateralFilterStep
import org.webcam_visual.preproc.PreprocPipeline
import org.webcam_visual.preproc.TemporalDenoiserStep
import org.webcam_visual.tracker.BlockTracker
import org.webcam_visual.tracker.OpticFlowBlockTracker
import kotlin.math.max
import kotlin.math.min
import org.firstinspires.ftc.robotcore.external.hardware.camera.*
import org.firstinspires.ftc.vision.VisionPortal
import org.opencv.core.Mat
import org.webcam_visual.common.FrameCtx

class AutoRobot(//public boolean SecondDriver = false;
    var hardwareMap: HardwareMap
) {
    val cameraHelper = CameraHelper(hardwareMap)
    var blockList: List<Block> = ArrayList()
        get() {
            synchronized(field) {
                return field
            }
        }
        private set(value) {
            synchronized(field) {
                field = value
            }
        }
    fun startVisionThread(): Thread {

        val visionThread = Thread {
            val pipeline = RobotVisionPipeline(
                PreprocPipeline(
                    BilateralFilterStep(
                        11, 55.0, 5.0, false
                    ),
                    TemporalDenoiserStep(.8, 30.0, 20)
                ),
                ColorBlockDetector(COLOR_DEF_R9000P),
                OpticFlowBlockTracker(),
                null
            )
            val frame = Mat()
            while (!Thread.interrupted()) {
                val frame = cameraHelper.getCurrentFrameMat()
                val ctx = pipeline.updateFrame(frame!!)
                blockList = ctx.curBlocks!!
            }
        }
        visionThread.start()
        return visionThread
    }

    var timer: ElapsedTime = ElapsedTime()
    var arm: AutoArm = AutoArm()
    var chassis: AutoChassis = AutoChassis(hardwareMap)
    var endPos: Pose2d? = null

    fun Autoinit(hwm: HardwareMap) {
        arm.autoInit(hwm)
        timer.reset()
        chassis.AutoInit(hwm)
        arm.closeClaw()
    }

    fun Teleinit(hwm: HardwareMap?) {
        chassis.TeleInit(hwm)
        arm.teleInit(hwm)
        timer.reset()
        //v.apt();
    }

    fun trackBlock(block: Block?) {
        if(block == null) {
            return
        }
        val unitToArmMovement = 100.0f
        val unitToWheelMovement = 0.8f
        val armCurrentPos = -arm.hzFront.currentPosition
        val nextPosition = (block.center.second * unitToArmMovement).toInt() + armCurrentPos
        arm.HzArmSet(max(0.0, min(1450.0, nextPosition.toDouble())).toInt())
        val motorPower = (block.center.first * unitToWheelMovement).toDouble()
        chassis.drive.setMotorPowers(motorPower, -motorPower, -motorPower, motorPower)
        sleep(200)
        chassis.drive.setMotorPowers(0.0, 0.0, 0.0, 0.0)
    }

    private fun sleep(milliseconds: Long) {
        try {
            Thread.sleep(milliseconds)
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        }
    } //    public void updatevisualposMT1(){
    //        c.posvisual(v.getfieldposeMT1());
    //    }
    //
    //    public void updatevisualposMT2(){
    //        c.posvisual(v.getfieldposeMT2());
    //    }

    companion object {
        fun getBlockAtCenter(blocks: List<Block>): Block? {
            var ans: Block? = null
            for (block in blocks) {
                if (ans == null) {
                    ans = block
                } else {
                    val center = block.center
                    val ansCenter = ans.center
                    if (center.first * center.first + center.second * center.second
                        < ansCenter.first * ansCenter.first + ansCenter.second * ansCenter.second
                    ) {
                        ans = block
                    }
                }
            }
            return ans
        }
    }
}
