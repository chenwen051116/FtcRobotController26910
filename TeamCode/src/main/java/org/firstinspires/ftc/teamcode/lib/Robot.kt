package org.firstinspires.ftc.teamcode.lib

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Auto.AutoRobot
import org.firstinspires.ftc.teamcode.ext.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.lib.schedule.Scheduler
import org.webcam_visual.common.Block
import org.webcam_visual.common.COLOR_DEF_R9000P
import org.webcam_visual.detectors.ColorBlockDetector
import org.webcam_visual.pipeline.RobotVisionPipeline
import org.webcam_visual.preproc.BilateralFilterStep
import org.webcam_visual.preproc.PreprocPipeline
import org.webcam_visual.preproc.TemporalDenoiserStep
import org.webcam_visual.tracker.OpticFlowBlockTracker
import org.yourorg.yourpackage.CameraHelper

class Robot(//public boolean SecondDriver = false;
    var hardwareMap: HardwareMap, scheduler: Scheduler?
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
            while (!Thread.interrupted()) {
                val frame = cameraHelper.getLatestFrame()
                val ctx = pipeline.updateFrame(frame!!)
                blockList = ctx.curBlocks!!
            }
        }
        visionThread.start()
        return visionThread
    }

    var timer: ElapsedTime = ElapsedTime()
    @JvmField
    var arm: Arm = Arm()
    @JvmField
    var chassis: Chassis = Chassis(hardwareMap)
    private var telemetry: Telemetry? = null

    var trajectorybar: TrajectorySequence? = null
    var trajectorybarback: TrajectorySequence? = null

    //public Visual v = new Visual();
    init {
        arm.setScheduler(scheduler)
    }

    fun autoInit(hwm: HardwareMap?) {
        arm.autoInit(hwm)
        timer.reset()
        chassis.AutoInit(hwm)
    }

    fun teleInit(hwm: HardwareMap, telemetry: Telemetry?) {
        chassis.TeleInit(hwm)
        arm.teleInit(hwm)
        timer.reset()
        this.telemetry = telemetry

        trajectorybar = chassis.drive.trajectorySequenceBuilder(Pose2d(0.0, 0.0, 0.0))
            .UNSTABLE_addTemporalMarkerOffset(0.0) {
                arm.highBar() //把arm伸上去
                chassis.turnAutoMode()
            }
            .waitSeconds(0.2)
            .lineToLinearHeading(Pose2d(28.8, 31.72, 3.14))
            .build()

        trajectorybarback = chassis.drive.trajectorySequenceBuilder(Pose2d(28.8, 31.72, 3.14))
            .UNSTABLE_addTemporalMarkerOffset(0.0) {
                chassis.turnAutoMode()
            }
            .lineToLinearHeading(Pose2d(0.0, 0.0, 0.0))
            .UNSTABLE_addTemporalMarkerOffset(-2.0) {
                arm.takeSpePos() //把arm伸上去
            }

            .build()
        //v.apt();
    }

    fun barAuto() {
        chassis.drive.followTrajectorySequenceAsync(trajectorybar)
        chassis.turnTeleMode()
    }

    fun barbackAuto() {
        chassis.drive.followTrajectorySequenceAsync(trajectorybarback)
        chassis.turnTeleMode()
    }

    fun alignClaw() {
        val block = AutoRobot.getBlockAtCenter(blockList)
        if(block != null) {
            val angle = block.angle
            arm.inClaw.position = (angle / 180).toDouble()
        }
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
}
