package org.firstinspires.ftc.teamcode.lib.vision.utils

import kotlin.math.sqrt
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import org.firstinspires.ftc.teamcode.lib.vision.common.HsvColorRange
/**
 * Extension function on FloatArray to compute its standard deviation.
 */
fun FloatArray.stdDev(): Float {
    val mean = this.average().toFloat()
    val variance = this.fold(0f) { acc, value -> acc + (value - mean) * (value - mean) } / this.size
    return sqrt(variance.toDouble()).toFloat()
}

/**
 * Extension function on FloatArray that computes the standard deviation for hue values,
 * taking into account hue wrapping.
 *
 * It computes the standard deviation of the original values, then creates a copy where
 * every value below [flipThreshold] is increased by 180, and computes the standard deviation of that copy.
 * The function returns the smaller of the two standard deviations.
 *
 * @param flipThreshold The threshold below which hue values are "flipped" by adding 180 (default is 90.0f).
 * @return The minimum standard deviation between the original and the shifted array.
 */
fun FloatArray.computeHueStdFlip(flipThreshold: Float = 90.0f): Float {
    // Compute standard deviation directly.
    val std1 = this.stdDev()

    // Create a shifted copy where values below the threshold are increased by 180.
    val shifted = this.copyOf()
    for (i in shifted.indices) {
        if (shifted[i] < flipThreshold) {
            shifted[i] += 180.0f
        }
    }
    val std2 = shifted.stdDev()

    return minOf(std1, std2)
}

fun createColorMask(
    frameHsv: Mat,
    colors: List<HsvColorRange>,
    maskDilateKernel: Mat = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.0, 3.0)),
    maskDilateIter: Int = 1
): Mat {
    // Create an empty mask with the same size as the input frame.
    val mask = Mat.zeros(frameHsv.rows(), frameHsv.cols(), CvType.CV_8UC1)

    // Iterate through each color range.
    for (color in colors) {
        for ((lower, upper) in color.hsvRanges) {
            val lowerScalar = Scalar(lower.first.toDouble(), lower.second.toDouble(), lower.third.toDouble())
            val upperScalar = Scalar(upper.first.toDouble(), upper.second.toDouble(), upper.third.toDouble())

            val tempMask = Mat()
            // Create a temporary mask for the current HSV range.
            Core.inRange(frameHsv, lowerScalar, upperScalar, tempMask)
            // Combine the temporary mask with the overall mask using bitwise OR.
            Core.bitwise_or(mask, tempMask, mask)
        }
    }

    // Apply dilation to the mask using the provided kernel and iteration count.
    Imgproc.dilate(mask, mask, maskDilateKernel, Point(-1.0, -1.0), maskDilateIter)
    return mask
}
