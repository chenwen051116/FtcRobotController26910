package org.firstinspires.ftc.teamcode.lib.vision

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

@Serializable
data class ColorData(val name: String, val bgr: Array<Int>, @SerialName("hsv_ranges") val hsvRanges: Array<Array<Array<Int>>>)