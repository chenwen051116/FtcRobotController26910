package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.lib.ArmKt
import org.firstinspires.ftc.teamcode.lib.ChassisKt
import org.firstinspires.ftc.teamcode.lib.RobotKt
import org.firstinspires.ftc.teamcode.lib.shLinOp

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
class TeleOpKt : LinearOpMode() {
    private val endPos = Pose2d(-34.9, 56.9, Math.toRadians(270.0))
    lateinit var robot: RobotKt
    override fun runOpMode() {
        // Initialize the robot components
        shLinOp = this
        robot = RobotKt(endPos, true)
        ArmKt.teleInit()

        // Initialize chassis
        waitForStart()
        while (opModeIsActive()) {
            // Gamepad1 controls
            handleGamepad1Controls()

            // Gamepad2 controls
            handleGamepad2Controls()
        }
    }

    private fun handleGamepad1Controls() {
        // Adjust chassis speed
        if (gamepad1.left_bumper || gamepad1.a) {
            ChassisKt.SpeedState.SLOW.setTo()
        } else {
            ChassisKt.SpeedState.NORMAL.setTo()
        }

        // Chassis motion
        val x = gamepad1.left_stick_x.toDouble()
        val y = gamepad1.left_stick_y.toDouble()
        val rx = gamepad1.right_stick_x.toDouble()
        ChassisKt.setMotion(x, y, rx)

        // Horizontal slider adjustment
        ArmKt.hzSliderLenAdj += (gamepad1.right_stick_y * 10).toInt()
    }

    private fun handleGamepad2Controls() {
        // Front intake control
        if (gamepad2.y) {
            ArmKt.FrSpFlipperState.DOWN.moveTo()
            ArmKt.FrSpIntakeState.IN.moveTo()
        } else {
            ArmKt.FrSpIntakeState.STOP.moveTo()
        }
        if (gamepad2.a) {
            ArmKt.FrSpFlipperState.UP.moveTo()
            ArmKt.FrSpIntakeState.STOP.moveTo()
        }

        // Back claw and dropping
        if (gamepad2.x) {
            ArmKt.BkSpClawState.CLOSE.moveTo()
        }
        if (gamepad2.b) {
            ArmKt.dropSpOnClaw()
        }

        // Back flipper position
        if (gamepad2.dpad_left) {
            ArmKt.BkSpFlipperState.UP.moveTo()
        }
        if (gamepad2.dpad_right) {
            ArmKt.BkSpFlipperState.DOWN.moveTo()
        }

        // Intake outward movement
        if (gamepad2.dpad_up || gamepad2.a) {
            ArmKt.FrSpIntakeState.OUT.moveTo()
        }

        // Vertical slider height control
        handleVerticalSliderControl()
    }

    private fun handleVerticalSliderControl() {
        val rightStickY = gamepad2.right_stick_y
        when {
            gamepad2.left_bumper -> when {
                rightStickY > 0.8 -> ArmKt.VtSliderHeiState.HIGH_BASKET.moveTo()
                rightStickY < -0.8 -> ArmKt.VtSliderHeiState.LOW_BASKET.moveTo()
                rightStickY in -0.2..0.2 -> ArmKt.VtSliderHeiState.LOWEST.moveTo()
            }

            gamepad2.right_bumper -> when {
                rightStickY > 0.8 -> ArmKt.VtSliderHeiState.HIGH_BAR.moveTo()
                rightStickY < -0.8 -> ArmKt.VtSliderHeiState.LOW_BAR.moveTo()
                rightStickY in -0.2..0.2 -> ArmKt.VtSliderHeiState.SIDE_SP.moveTo()
            }
        }
    }
}
