package org.webcam_visual.preproc

import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.webcam_visual.common.FrameCtx
import org.webcam_visual.utils.mat.div
import org.webcam_visual.utils.mat.minus
import org.webcam_visual.utils.mat.times

// ------------------------
// Auto White Balance Step
// ------------------------
/**
 * AutoWhiteBalanceStep attempts to adjust white balance automatically.
 * If the advanced API is available, it uses that; otherwise, it falls back to a simple Lab-based correction.
 */
class AutoWhiteBalanceStep(initDebug: Boolean = false) : PreprocStep("auto_wb", initDebug) {
    override fun _process(ctx: FrameCtx): Mat {
        val image = ctx.frame!!
        return try {
            val xphotoClass = Class.forName("org.opencv.xphoto.Xphoto")
            val createSimpleWBMethod = xphotoClass.getMethod("createSimpleWB")
            val wb = createSimpleWBMethod.invoke(null)
            val balanceWhiteMethod = wb.javaClass.getMethod("balanceWhite", Mat::class.java)
            balanceWhiteMethod.invoke(wb, image) as Mat
        } catch (e: Exception) {
            // Fallback implementation using Lab conversion
            val lab = Mat()
            Imgproc.cvtColor(image, lab, Imgproc.COLOR_BGR2Lab)
            val labChannels = ArrayList<Mat>()
            Core.split(lab, labChannels)  // labChannels[0]=L, [1]=a, [2]=b
            val avgA = Core.mean(labChannels[1]).`val`[0]
            val avgB = Core.mean(labChannels[2]).`val`[0]
            val LDiv = labChannels[0] / Scalar(255.0)
            val adjustA = LDiv * Scalar(avgA - 128.0)
            val newA = labChannels[1] - adjustA
            val adjustB = LDiv * Scalar(avgB - 128.0)
            val newB = labChannels[2] - adjustB
            Core.merge(listOf(labChannels[0], newA, newB), lab)
            Imgproc.cvtColor(lab, image, Imgproc.COLOR_Lab2BGR)
            image
        }
    }
}

// ---------------------
// Brightness Adjustment
// ---------------------
/**
 * BrightnessStep adjusts the brightness by adding a constant value.
 */
class BrightnessStep(private val brightness: Int, initDebug: Boolean = false) : PreprocStep("brightness", initDebug) {
    override fun _process(ctx: FrameCtx): Mat {
        val image = ctx.frame!!
        val result = Mat()
        Core.convertScaleAbs(image, result, 1.0, brightness.toDouble())
        return result
    }
}

// -------------------------
// Histogram Equalization
// -------------------------
/**
 * HistEqualizeStep performs histogram equalization.
 * For grayscale images, it applies equalization directly.
 * For color images, it converts to YCrCb, equalizes the Y channel, then converts back.
 */
class HistEqualizeStep(initDebug: Boolean = false) : PreprocStep("hist_equalize", initDebug) {
    override fun _process(ctx: FrameCtx): Mat {
        val image = ctx.frame!!
        val result = if (image.channels() == 1) {
            val tmp = Mat()
            Imgproc.equalizeHist(image, tmp)
            tmp
        } else {
            val ycrcb = Mat()
            Imgproc.cvtColor(image, ycrcb, Imgproc.COLOR_BGR2YCrCb)
            val channels = ArrayList<Mat>()
            Core.split(ycrcb, channels)
            Imgproc.equalizeHist(channels[0], channels[0])
            Core.merge(channels, ycrcb)
            val tmp = Mat()
            Imgproc.cvtColor(ycrcb, tmp, Imgproc.COLOR_YCrCb2BGR)
            tmp
        }
        return result
    }
}

// -------------------------
// CLAHE (Contrast Limited Adaptive Histogram Equalization)
// -------------------------
/**
 * CLAHEStep applies CLAHE to enhance contrast.
 */
class CLAHEStep(private val clipLimit: Double, private val tileGridSize: Size, initDebug: Boolean = false)
    : PreprocStep("clahe", initDebug) {
    override fun _process(ctx: FrameCtx): Mat {
        val image = ctx.frame!!
        val clahe = Imgproc.createCLAHE(clipLimit, tileGridSize)
        val result = if (image.channels() == 1) {
            val tmp = Mat()
            clahe.apply(image, tmp)
            tmp
        } else {
            val lab = Mat()
            Imgproc.cvtColor(image, lab, Imgproc.COLOR_BGR2Lab)
            val channels = ArrayList<Mat>()
            Core.split(lab, channels)
            clahe.apply(channels[0], channels[0])
            Core.merge(channels, lab)
            val tmp = Mat()
            Imgproc.cvtColor(lab, tmp, Imgproc.COLOR_Lab2BGR)
            tmp
        }
        return result
    }
}

// ---------------------
// Gaussian Blur
// ---------------------
/**
 * GaussianBlurStep applies Gaussian blur to the image.
 */
class GaussianBlurStep(private val ksize: Int, private val sigma: Double, initDebug: Boolean = false)
    : PreprocStep("gaussian", initDebug) {
    override fun _process(ctx: FrameCtx): Mat {
        val image = ctx.frame!!
        val result = Mat()
        Imgproc.GaussianBlur(image, result, Size(ksize.toDouble(), ksize.toDouble()), sigma)
        return result
    }
}

// ---------------------
// Median Blur
// ---------------------
/**
 * MedianBlurStep applies a median blur with the given kernel size.
 */
class MedianBlurStep(private val kernelSize: Int, initDebug: Boolean = false)
    : PreprocStep("median", initDebug) {
    override fun _process(ctx: FrameCtx): Mat {
        val image = ctx.frame!!
        val result = Mat()
        Imgproc.medianBlur(image, result, kernelSize)
        return result
    }
}

// --------------------------
// Bilateral Filter
// --------------------------
/**
 * BilateralFilterStep applies a bilateral filter to preserve edges.
 */
class BilateralFilterStep(
    private val d: Int,
    private val sigmaColor: Double,
    private val sigmaSpace: Double,
    initDebug: Boolean = false
) : PreprocStep("bilateral", initDebug) {
    override fun _process(ctx: FrameCtx): Mat {
        val image = ctx.frame!!
        val result = Mat()
        Imgproc.bilateralFilter(image, result, d, sigmaColor, sigmaSpace)
        return result
    }
}

// --------------------------
// Guided Filter
// --------------------------
/**
 * GuidedFilterStep attempts to apply a guided filter.
 * If unavailable, it falls back to a bilateral filter.
 */
class GuidedFilterStep(private val radius: Int, private val eps: Double, initDebug: Boolean = false)
    : PreprocStep("guided", initDebug) {
    override fun _process(ctx: FrameCtx): Mat {
        val image = ctx.frame!!
        val result = try {
            val ximgprocClass = Class.forName("org.opencv.ximgproc.Ximgproc")
            val guidedFilterMethod = ximgprocClass.getMethod("guidedFilter", Mat::class.java, Mat::class.java, Int::class.java, Double::class.java)
            guidedFilterMethod.invoke(null, image, image, radius, eps) as Mat
        } catch (e: Exception) {
            println("Warning: Guided filtering unavailable. Using bilateral filter instead.")
            val fallback = Mat()
            Imgproc.bilateralFilter(image, fallback, 7, 75.0, 5.0)
            fallback
        }
        return result
    }
}

// --------------------------
// Laplacian Filter (Sharpening)
// --------------------------
/**
 * LaplacianStep applies a Laplacian filter and blends the result to sharpen the image.
 */
class LaplacianStep(
    private val kernelSize: Int,
    private val scale: Double,
    private val weight: Double,
    initDebug: Boolean = false
) : PreprocStep("laplacian", initDebug) {
    override fun _process(ctx: FrameCtx): Mat {
        val image = ctx.frame!!
        val lap = Mat()
        Imgproc.Laplacian(image, lap, CvType.CV_64F, kernelSize, scale)
        val absLap = Mat()
        Core.convertScaleAbs(lap, absLap)
        val sharpened = Mat()
        Core.addWeighted(image, 1.0 + weight, absLap, -weight, 0.0, sharpened)
        val result = Mat()
        Core.min(sharpened, Scalar(255.0), result)
        Core.max(result, Scalar(0.0), result)
        result.convertTo(result, CvType.CV_8U)
        return result
    }
}

// --------------------------
// Morphological Opening
// --------------------------
/**
 * MorphOpenStep applies a morphological opening operation.
 */
class MorphOpenStep(
    private val kernelSize: Int,
    private val iterations: Int,
    initDebug: Boolean = false
) : PreprocStep("morph_open", initDebug) {
    override fun _process(ctx: FrameCtx): Mat {
        val image = ctx.frame!!
        val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(kernelSize.toDouble(), kernelSize.toDouble()))
        val result = Mat()
        Imgproc.morphologyEx(image, result, Imgproc.MORPH_OPEN, kernel, Point(-1.0, -1.0), iterations)
        return result
    }
}

// --------------------------
// Morphological Closing
// --------------------------
/**
 * MorphCloseStep applies a morphological closing operation.
 */
class MorphCloseStep(
    private val kernelSize: Int,
    private val iterations: Int,
    initDebug: Boolean = false
) : PreprocStep("morph_close", initDebug) {
    override fun _process(ctx: FrameCtx): Mat {
        val image = ctx.frame!!
        val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(kernelSize.toDouble(), kernelSize.toDouble()))
        val result = Mat()
        Imgproc.morphologyEx(image, result, Imgproc.MORPH_CLOSE, kernel, Point(-1.0, -1.0), iterations)
        return result
    }
}

// --------------------------
// USM Sharpening
// --------------------------
/**
 * USMSharpeningStep applies unsharp masking to sharpen the image.
 */
class USMSharpeningStep(
    private val kernelSize: Int,
    private val sigma: Double,
    private val weight: Double,
    initDebug: Boolean = false
) : PreprocStep("usm", initDebug) {
    override fun _process(ctx: FrameCtx): Mat {
        val image = ctx.frame!!
        val blurred = Mat()
        Imgproc.GaussianBlur(image, blurred, Size(kernelSize.toDouble(), kernelSize.toDouble()), sigma)
        val sharpened = Mat()
        Core.addWeighted(image, 1.0 + weight, blurred, -weight, 0.0, sharpened)
        val result = Mat()
        Core.min(sharpened, Scalar(255.0), result)
        Core.max(result, Scalar(0.0), result)
        result.convertTo(result, CvType.CV_8U)
        return result
    }
}

// --------------------------
// Image Scaling
// --------------------------
/**
 * ScaleStep resizes the image using the given scaling factors and interpolation method.
 */
class ScaleStep(
    private val factorX: Double,
    private val factorY: Double,
    private val interp: Int,
    initDebug: Boolean = false
) : PreprocStep("scale", initDebug) {
    override fun _process(ctx: FrameCtx): Mat {
        val image = ctx.frame!!
        val result = Mat()
        Imgproc.resize(image, result, Size(), factorX, factorY, interp)
        return result
    }
}
