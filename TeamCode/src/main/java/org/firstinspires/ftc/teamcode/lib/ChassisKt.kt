package org.firstinspires.ftc.teamcode.lib

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.ext.roadrunner.drive.SampleMecanumDrive
import kotlin.math.abs

object ChassisKt {
    var isTele: Boolean = true
    var lastPos: Pose2d = Pose2d(0.0, 0.0, 0.0)
    val mecDrive: SampleMecanumDrive
    var powerCoef: Double = 1.0

    init {
        mecDrive = SampleMecanumDrive(shLinOp!!.hardwareMap)
        mecDrive.poseEstimate = lastPos
    }

    enum class SpeedState(val powerCoef: Double) {
        SLOW(0.3),
        NORMAL(1.0);

        fun setTo() {
            ChassisKt.powerCoef = this.powerCoef
        }
    }

    fun setMotion(x: Double, y: Double, rot: Double) {
        val leftFront = ((y - x - rot) * powerCoef)
        val leftRear = ((y + x - rot) * powerCoef)
        val rightFront = ((y - x + rot) * powerCoef)
        val rightRear = ((y + x + rot) * powerCoef)
        val mxSpeed =
            maxOf(maxOf(abs(leftFront), abs(leftRear)), maxOf(abs(rightFront), abs(rightRear)))
        if (mxSpeed > 1.0) {
            mecDrive.setMotorPowers(
                leftFront / mxSpeed,
                leftRear / mxSpeed,
                rightFront / mxSpeed,
                rightRear / mxSpeed
            )
        } else {
            mecDrive.setMotorPowers(leftFront, leftRear, rightFront, rightRear)
        }
    }
}