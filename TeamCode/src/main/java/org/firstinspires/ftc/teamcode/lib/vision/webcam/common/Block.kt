package org.webcam_visual.common
import org.opencv.core.MatOfPoint
import org.webcam_visual.common.HsvColorRange
import org.webcam_visual.common.HsvColorStats

data class Block(
    val center: Pair<Float, Float>,
    val size: Pair<Float, Float>,
    val angle: Float,
    val color: HsvColorRange,
    val colorStd: HsvColorStats,
    val colorMean: HsvColorStats,
    val contour: MatOfPoint, // Stores contour as a set of points
    var relativeCenter: Pair<Float, Float> = Pair(0f, 0f),
    var relativeSize: Pair<Float, Float> = Pair(0f, 0f),
    val id: Int = -1, // positive only after tracking is done
) {
    /**
     * Updates `relativeCenter` and `relativeSize` based on the given frame dimensions.
     * This modifies the object in place instead of returning a new instance.
     */
    fun calcRelative(frameWidth : Int, frameHeight : Int) {
        relativeCenter = Pair(
            (center.first - frameWidth / 2) / frameWidth,
            (center.second - frameHeight / 2) / frameHeight
        )
        relativeSize = Pair(
            size.first / frameWidth,
            size.second / frameHeight
        )
    }
}