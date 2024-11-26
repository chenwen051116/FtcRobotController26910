package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.lib.ArmKt
import org.firstinspires.ftc.teamcode.lib.ChassisKt
import org.firstinspires.ftc.teamcode.lib.RobotKt
import org.firstinspires.ftc.teamcode.lib.shLinOp
import org.firstinspires.ftc.teamcode.lib.toRad

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
class TeleOpKt : LinearOpMode() {
    val lastPos = Pose2d(-34.9, 56.9, 270.0.toRad())
    lateinit var robot : RobotKt
    override fun runOpMode() {
        shLinOp = this
        robot = RobotKt(lastPos, true)
        waitForStart()

        while (opModeIsActive()) {
            // Adjust Chassis Speed
            if (gamepad1.left_bumper){
                ChassisKt.SpeedState.SLOW.setTo()
            } else {
                ChassisKt.SpeedState.NORMAL.setTo()
            }

            // Chassis Motion
            ChassisKt.setMotion(
                x = gamepad1.left_stick_x.toDouble(),
                y = gamepad1.left_stick_y.toDouble(),
                rot = gamepad1.right_stick_x.toDouble()
            )

            // Horizontal Slider Adjustment
            ArmKt.hzSliderLenAdj -= (gamepad1.right_stick_y * 10).toInt()

            // Gamepad2 Controls
            handleGamepad2Controls()
        }
    }

    // Extracted Function to Handle Gamepad2 Controls
    private fun handleGamepad2Controls() {
        // Front Flipper and Intake
        if (gamepad2.y) {
            ArmKt.FrSpFlipperState.DOWN.moveTo()
            ArmKt.FrSpIntakeState.IN.moveTo()
        }

        if (gamepad2.a) {
            ArmKt.FrSpFlipperState.UP.moveTo()
            ArmKt.FrSpIntakeState.STOP.moveTo()
        }

        // Back Claw and Dropping
        if (gamepad2.x) ArmKt.BkSpClawState.CLOSE.moveTo()
        if (gamepad2.b) ArmKt.dropSpOnClaw()

        // Back Flipper Position
        if (gamepad2.dpad_left) ArmKt.BkSpFlipperState.UP.moveTo()
        if (gamepad2.dpad_right) ArmKt.BkSpFlipperState.DOWN.moveTo()

        // Intake Outward Movement
        if (gamepad2.dpad_up || gamepad2.a) ArmKt.FrSpIntakeState.OUT.moveTo()

        // Vertical Slider Height Control
        handleVerticalSliderControl()
    }

    // Extracted Function for Vertical Slider Control
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