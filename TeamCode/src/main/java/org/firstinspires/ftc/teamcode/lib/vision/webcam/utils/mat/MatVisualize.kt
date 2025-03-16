package org.webcam_visual.utils.mat

import org.opencv.core.*
import org.opencv.imgproc.Imgproc

/**
 * Applies an OpenCV colormap to a single-channel floating-point Mat and generates a corresponding legend bar.
 *
 * This function normalizes the input matrix to the range [0,255], converts it to 8-bit, and applies the specified colormap.
 * It then creates a vertical legend bar that visually represents the mapping between intensity values and the resulting colors.
 *
 * @param srcMat       A single-channel CV_32F Mat containing the values to be visualized.
 * @param colormap     An OpenCV colormap identifier (e.g., Imgproc.COLORMAP_JET).
 * @param legendHeight The height (in pixels) of the legend bar.
 * @param legendWidth  The width (in pixels) of the legend bar (default 30).
 * @return A Pair where the first element is the color-mapped image and the second is the generated legend bar.
 */
fun createColorMappedImageWithLegend(
    distMat: Mat,
    colormap: Int = Imgproc.COLORMAP_JET,
    barHeight: Int = 200,
    barWidth: Int = 30
): Pair<Mat, Mat> {

    // 1) Determine min/max of distMat
    val mm = Core.minMaxLoc(distMat)
    val minVal = mm.minVal
    val maxVal = mm.maxVal

    // 2) Normalize distMat to [0..255] and convert to 8U
    //    If maxVal == minVal, we artificially expand the range to avoid a divide by zero
    val norm = Mat()
    Core.normalize(distMat, norm, 0.0, 255.0, Core.NORM_MINMAX)
    norm.convertTo(norm, CvType.CV_8U)

    // 3) Apply the chosen color map
    val coloredDist = Mat()
    Imgproc.applyColorMap(norm, coloredDist, colormap)

    // 4) Create a vertical gradient from [0..255] to represent the color scale
    val gradient = Mat(barHeight, 1, CvType.CV_8U)
    for (row in 0 until barHeight) {
        // row=0 => intensity=255 at top, row=barHeight-1 => intensity=0 at bottom
        val intensity = 255 - (row * 255 / (barHeight - 1))
        gradient.put(row, 0, intensity.toDouble())
    }

    // 5) Apply the same color map to the gradient to get a color bar
    val coloredBar = Mat()
    Imgproc.applyColorMap(gradient, coloredBar, colormap)

    // 6) Resize the 1-column bar to the desired barWidth
    val colorBar = Mat()
    Imgproc.resize(coloredBar, colorBar, Size(barWidth.toDouble(), barHeight.toDouble()))

    // 7) Optional: label the min/max values on the bar
    //    We'll put them in white near the top & bottom
    //    (Because the bar is vertical, top = maxVal, bottom = minVal)
    val fontFace = Imgproc.FONT_HERSHEY_SIMPLEX
    val fontScale = 0.4
    val white = Scalar(255.0, 255.0, 255.0)

    // String formatting with 1 decimal place
    val maxText = String.format("%.1f", maxVal)
    val minText = String.format("%.1f", minVal)
    // Draw max near top, offset a bit from the left
    Imgproc.putText(colorBar, maxText, Point(2.0, 12.0), fontFace, fontScale, white)
    // Draw min near bottom
    Imgproc.putText(colorBar, minText, Point(2.0, barHeight - 5.0), fontFace, fontScale, white)

    return coloredDist to colorBar
}


/**
 * Draws optical flow vectors as arrows on a blank image.
 *
 * @param flow The optical flow matrix (CV_32FC2).
 * @param step The grid step size for sampling flow vectors.
 * @return A visualization image (CV_8UC3) with arrows drawn.
 */
fun drawFlowArrows(flow: Mat, step: Int): Mat {
    // Create a blank image (black) with same size as flow.
    val vis = Mat.zeros(flow.size(), CvType.CV_8UC3)
    val rows = flow.rows()
    val cols = flow.cols()
    for (y in 0 until rows step step) {
        for (x in 0 until cols step step) {
            val flowAt = flow.get(y, x)
            val dx = flowAt[0]
            val dy = flowAt[1]
            val pt1 = Point(x.toDouble(), y.toDouble())
            val pt2 = Point(x + dx, y + dy)
            // Draw an arrowed line in green.
            Imgproc.arrowedLine(vis, pt1, pt2, Scalar(0.0, 255.0, 0.0), 1)
        }
    }
    return vis
}