package org.webcam_visual.detectors

import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.webcam_visual.common.*
import org.webcam_visual.utils.createColorMask
import org.webcam_visual.utils.computeHueStdFlip
import org.webcam_visual.utils.stdDev

/**
 * Detects color blocks by:
 *   1) Converting the (preprocessed) frame to HSV
 *   2) Creating color masks for each color definition
 *   3) Finding contours and computing mean/std of H, S, V in each ROI
 *   4) Creating a Block if the statistics are within threshold.
 *
 * Debug options (available to the GUI):
 *   - "color_mask_<colorName>": the mask for that specific color.
 *   - "combined_color_mask": the combined mask for all colors.
 */
class ColorDetector(
    private val detectingColors: List<HsvColorRange>,
    private val config: Cfg = Cfg()
) : BlockDetector, DefaultImgDebuggable() {

    // Internal configuration for the detector.
    data class Cfg(
        val minContourArea: Double = 500.0,
        val stdThresholdHsv: HsvColorStats = Triple(40f, 50f, 50f),
        val maskDilateIter: Int = 2,
        val maskDilateKernel: Mat = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, Size(7.0, 7.0))
    )

    init {
        // Register a debug option for each individual color mask.
        detectingColors.forEach { color ->
            setDbgOption("color_mask_${color.name}", false)
        }
        // Register the combined color mask debug option.
        setDbgOption("combined_color_mask", false)
    }

    /**
     * Assumes that the frame has been preprocessed already.
     * Converts the frame to HSV and then detects blocks.
     */
    fun processFrame(frame: Mat): List<Block> {
        val hsv = Mat()
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV)
        return detectBlocks(hsv)
    }

    /**
     * Implementation of the BlockDetector interface.
     */
    override fun detectBlocks(ctx: FrameCtx): FrameCtx {
        return ctx.copy(curBlocks = processFrame(ctx.frame))
    }

    /**
     * For each color definition:
     *   - Create a color mask.
     *   - If its debug option is enabled, create a BGR version of the mask and store it.
     *   - Combine all masks for a combined debug image.
     *   - Find contours in the mask and process them.
     */
    private fun detectBlocks(hsv: Mat): List<Block> {
        val blocks = mutableListOf<Block>()
        val combinedMask = Mat.zeros(hsv.rows(), hsv.cols(), CvType.CV_8UC1)

        for (colorDef in detectingColors) {
            // Create a color mask using the utility function.
            val mask = createColorMask(hsv, listOf(colorDef), config.maskDilateKernel, config.maskDilateIter)

            // If the debug option for this color is enabled, generate and store the debug image.
            val dbgKey = "color_mask_${colorDef.name}"
            if (isDbgOptionEnabled(dbgKey)) {
                val maskBgr = Mat()
                Imgproc.cvtColor(mask, maskBgr, Imgproc.COLOR_GRAY2BGR)
                val bgrColor = colorDef.bgr
                val colorScalar = Scalar(
                    bgrColor.first.toDouble(),
                    bgrColor.second.toDouble(),
                    bgrColor.third.toDouble()
                )
                // Set white pixels to the BGR color.
                maskBgr.setTo(colorScalar, mask)
                addDbgEntry(dbgKey, maskBgr)
            }

            // Merge with the combined mask.
            Core.bitwise_or(combinedMask, mask, combinedMask)

            // Find contours and process them.
            val contours = findContours(mask)
            val colorBlocks = processContours(contours, colorDef, hsv)
            blocks.addAll(colorBlocks)
        }

        // If enabled, store the combined color mask.
        if (isDbgOptionEnabled("combined_color_mask")) {
            addDbgEntry("combined_color_mask", combinedMask)
        }
        return blocks
    }

    /**
     * Finds external contours in the mask that exceed the minimum area.
     */
    private fun findContours(mask: Mat): List<MatOfPoint> {
        val contours = mutableListOf<MatOfPoint>()
        val hierarchy = Mat()
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        return contours.filter { Imgproc.contourArea(it) > config.minContourArea }
    }

    /**
     * For each contour, computes the ROI, extracts valid HSV values, and computes the mean and std.
     * A Block is created only if the standard deviations for H, S, and V are within thresholds.
     */
    private fun processContours(
        contours: List<MatOfPoint>,
        colorDef: HsvColorRange,
        hsv: Mat
    ): List<Block> {
        val blocks = mutableListOf<Block>()
        for (cnt in contours) {
            val cnt2f = MatOfPoint2f(*cnt.toArray())
            val rect = Imgproc.minAreaRect(cnt2f)
            val center = rect.center
            var w = rect.size.width
            var h = rect.size.height
            var angle = rect.angle

            // Normalize the orientation.
            if (w < h) {
                val tmp = w
                w = h
                h = tmp
                angle += 90.0
            }

            val boundingRect = Imgproc.boundingRect(cnt)
            if (boundingRect.width == 0 || boundingRect.height == 0) continue

            // Create a mask for the contour within its bounding box.
            val contourMask = Mat.zeros(boundingRect.height, boundingRect.width, CvType.CV_8UC1)
            val shiftedPoints = cnt.toArray().map { Point(it.x - boundingRect.x, it.y - boundingRect.y) }
                .toTypedArray()
            val shiftedContour = MatOfPoint(*shiftedPoints)
            Imgproc.drawContours(contourMask, listOf(shiftedContour), 0, Scalar(255.0), -1)

            // Extract the ROI from the HSV image.
            val hsvRoi = Mat(hsv, boundingRect)
            val hsvMasked = Mat()
            Core.bitwise_and(hsvRoi, hsvRoi, hsvMasked, contourMask)

            // Split channels.
            val channels = mutableListOf<Mat>()
            Core.split(hsvMasked, channels)
            if (channels.size < 3) continue
            val hCh = channels[0]
            val sCh = channels[1]
            val vCh = channels[2]

            // Collect valid HSV values within the contour.
            val hValid = mutableListOf<Float>()
            val sValid = mutableListOf<Float>()
            val vValid = mutableListOf<Float>()
            for (row in 0 until contourMask.rows()) {
                for (col in 0 until contourMask.cols()) {
                    if (contourMask.get(row, col)[0] == 255.0) {
                        hValid.add(hCh.get(row, col)[0].toFloat())
                        sValid.add(sCh.get(row, col)[0].toFloat())
                        vValid.add(vCh.get(row, col)[0].toFloat())
                    }
                }
            }
            if (hValid.isEmpty()) continue

            val meanH = hValid.average().toFloat()
            val meanS = sValid.average().toFloat()
            val meanV = vValid.average().toFloat()

            val stdH = hValid.toFloatArray().computeHueStdFlip(90.0f)
            val stdS = sValid.toFloatArray().stdDev()
            val stdV = vValid.toFloatArray().stdDev()

            val (mxStdH, mxStdS, mxStdV) = config.stdThresholdHsv
            if (stdH <= mxStdH &&
                stdS <= mxStdS &&
                stdV <= mxStdV
            ) {
                val block = Block(
                    center = Pair(center.x.toFloat(), center.y.toFloat()),
                    size = Pair(w.toFloat(), h.toFloat()),
                    angle = angle.toFloat(),
                    color = colorDef,
                    colorStd = HsvColorStats(stdH, stdS, stdV),
                    colorMean = HsvColorStats(meanH, meanS, meanV),
                    contour = cnt
                )
                blocks.add(block)
            }
        }
        return blocks
    }
}
