package org.firstinspires.ftc.teamcode.lib.vision.common

import kotlin.math.sqrt

typealias HsvColor = Triple<Int, Int, Int>
typealias HsvColorStats = Triple<Float, Float, Float>
typealias RgbColor = Triple<Int, Int, Int>
typealias RgbColorStats = Triple<Float, Float, Float>
typealias BgrColor = Triple<Int, Int, Int>
typealias BgrColorStats = Triple<Float, Float, Float>
typealias GrayColor = Int
typealias GrayColorStats = Float

data class HsvColorRange(
    val name : String,
    val hsvRanges : List<Pair<HsvColor, HsvColor>>,
    // it is possible for hsv to have two ranges for one range in bgr
    // red as an example
    val bgr : BgrColor
)



val RED_R9000P = HsvColorRange(
    name = "RED_R9000P",
    hsvRanges = listOf(
        Pair(Triple(0, 70, 50), Triple(3, 160, 225)),
        Pair(Triple(165, 70, 50), Triple(180, 160, 225))
    ),
    bgr = Triple(0, 0, 255)
)

val BLUE_R9000P = HsvColorRange(
    name = "BLUE_R9000P",
    hsvRanges = listOf(
        Pair(Triple(110, 80, 70), Triple(125, 180, 230))
    ),
    bgr = Triple(255, 0, 0)
)

val YELLOW_R9000P = HsvColorRange(
    name = "YELLOW_R9000P",
    hsvRanges = listOf(
        Pair(Triple(17, 60, 140), Triple(32, 125, 255))
    ),
    bgr = Triple(0, 255, 255)
)

val COLOR_DEF_R9000P = listOf(RED_R9000P, BLUE_R9000P, YELLOW_R9000P)
