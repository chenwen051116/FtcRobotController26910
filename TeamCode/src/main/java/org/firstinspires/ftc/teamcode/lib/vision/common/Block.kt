package org.firstinspires.ftc.teamcode.lib.vision.common

import org.opencv.core.MatOfPoint

data class Block(
    val center: Pair<Float, Float>,
    val size: Pair<Float, Float>,
    val angle: Float,
    val color: HsvColorRange,
    val colorStd: HsvColorStats,
    val colorMean: HsvColorStats,
    val contour: MatOfPoint, // Stores contour as a set of points
    val id : Int = -1, // positive only after tracking is done
)