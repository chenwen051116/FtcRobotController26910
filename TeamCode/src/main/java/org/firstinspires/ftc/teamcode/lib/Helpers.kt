package org.firstinspires.ftc.teamcode.lib

fun telePrintln(str: String) {
    if (shLinOp != null) {
        shLinOp!!.telemetry.addLine(str)
        shLinOp!!.telemetry.update()
    }
}

fun telePrint(str: String) {
    if (shLinOp != null) {
        shLinOp!!.telemetry.addData("", str)
        shLinOp!!.telemetry.update()
    }
}