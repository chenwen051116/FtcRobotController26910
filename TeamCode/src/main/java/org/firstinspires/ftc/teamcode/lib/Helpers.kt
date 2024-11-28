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

inline fun <reified T> T.getFromHwMap(name: String): T {
    return shLinOp!!.hardwareMap.get(T::class.java, name)
}

fun Double.toRad() = this * Math.PI / 180
fun Double.toDeg() = this * 180 / Math.PI
fun Float.toRad() = this * Math.PI / 180
fun Float.toDeg() = this * 180 / Math.PI