package org.firstinspires.ftc.teamcode.lib

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.ext.roadrunner.drive.SampleMecanumDrive

object ChassisKt{
    var isTele : Boolean = true
    var lastPos : Pose2d = Pose2d(0.0, 0.0, 0.0)
    val MecDrive : SampleMecanumDrive;
    var powerCoef : Double = 1.0;
    init{
        MecDrive = SampleMecanumDrive(shLinOp!!.hardwareMap)
        MecDrive.poseEstimate = lastPos
    }

    enum class SpeedState(val powerCoef : Double){
        SLOW(0.3),
        NORMAL(1.0);
        fun setTo(){
            ChassisKt.powerCoef = this.powerCoef
        }
    }

    fun setMotion(x : Double, y : Double, rot : Double){
        MecDrive.setWeightedDrivePower(Pose2d(y * powerCoef, rot * powerCoef, x * powerCoef))
    }
}