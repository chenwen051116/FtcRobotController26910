package org.webcam_visual.utils.mat

import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc

fun Mat.grayToBGR(): Mat {
    val colorMat = Mat()
    Imgproc.cvtColor(this, colorMat, Imgproc.COLOR_GRAY2BGR)
    return colorMat
}
