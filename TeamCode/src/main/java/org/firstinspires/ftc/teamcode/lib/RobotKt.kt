package org.firstinspires.ftc.teamcode.lib

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.util.ElapsedTime

class RobotKt (
    val lastPos : Pose2d,
    var isTele : Boolean,
    val timer : ElapsedTime = ElapsedTime(),
    val arm : ArmKt = ArmKt,
    val chassis: ChassisKt = ChassisKt
){
    init {
        arm.isTele = isTele
        chassis.isTele = isTele
        chassis.lastPos = lastPos
    }
}