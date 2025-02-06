package org.firstinspires.ftc.teamcode.lib.vision

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json

@Serializable
data class BlockData(
    val center: Array<Float>,
    val size: Array<Float>,
    val angle: Float,
    val color: ColorData,
    @SerialName("color_std") val colorStd: Array<Float>,
    @SerialName("mean_hsv") val meanHSV: Array<Float>,
    val contour: Array<Float>,
) {
    companion object {
        fun deserialize(str: String): BlockData {
            return Json.decodeFromString(str)
        }
    }
}