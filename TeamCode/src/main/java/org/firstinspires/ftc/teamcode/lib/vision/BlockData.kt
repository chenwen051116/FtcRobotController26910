package org.firstinspires.ftc.teamcode.lib.vision

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

@Serializable
data class BlockData(
    val center: Array<Float>,
    val size: Array<Float>,
    val angle: Array<Float>,
    val color: ColorData,
    @SerialName("color_std") val colorStd: Array<Float>,
    @SerialName("mean_hsv") val meanHSV: Array<Float>,
    val contour: Array<Float>,
)