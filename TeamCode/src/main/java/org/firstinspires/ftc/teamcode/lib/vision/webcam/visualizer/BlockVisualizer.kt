package org.webcam_visual.visualizer

import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.RotatedRect
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import org.webcam_visual.common.Block
import org.webcam_visual.common.FrameCtx
import org.webcam_visual.common.ImgDebuggable

/**
 * Visualizes blocks by drawing contours, rotated bounding boxes and block info (color, angle, HSV stats)
 * onto a copy of the provided frame.
 */
class BlockVisualizer : ImgDebuggable {
    override val availableDbgOptions: MutableMap<String, Boolean> = mutableMapOf()
    override val dbgData: MutableMap<String, Mat> = mutableMapOf()
    override val dbgChildren: MutableList<ImgDebuggable> = mutableListOf()

    init {
        setDbgOption("blocks", true)
    }

    /**
     * Draws each block on a copy of the given frame.
     *
     * For each block:
     *   - Draw its contour.
     *   - Compute and draw its rotated bounding box.
     *   - Put text with block information: color name, angle, stdHSV and avgHSV.
     *
     * @param frame The input image (assumed preprocessed).
     * @param blocks The list of detected blocks.
     * @return A new Mat with all blocks visualized.
     */
    fun visualizeBlocks(ctx: FrameCtx): Mat {
        val frame = ctx.frame
        val blocks = ctx.curBlocks ?: return frame
        val output = frame.clone()

        for ((_, block) in blocks.withIndex()) {
            // Get the color for drawing from the block's color (assumes a Triple<Int, Int, Int> for BGR)
            val colorScalar = Scalar(
                block.color.bgr.first.toDouble(),
                block.color.bgr.second.toDouble(),
                block.color.bgr.third.toDouble()
            )

            // Draw the block's contour (assumes block.contour is a MatOfPoint)
            Imgproc.drawContours(output, listOf(block.contour), -1, colorScalar, 2)

            // Draw a rotated bounding box using block center, size and angle.
            val center = Point(block.center.first.toDouble(), block.center.second.toDouble())
            val size = Size(block.size.first.toDouble(), block.size.second.toDouble())
            val rect = RotatedRect(center, size, block.angle.toDouble())
            val boxPoints = arrayOfNulls<Point>(4)
            rect.points(boxPoints)
            // Draw lines connecting the box points.
            val pts = boxPoints.map { it!! }.toTypedArray()
            for (i in pts.indices) {
                Imgproc.line(output, pts[i], pts[(i + 1) % pts.size], colorScalar, 2)
            }

            // Prepare text lines with block information.
            val x0 = block.center.first.toInt()
            val y0 = block.center.second.toInt()

            val line1 = if (block.id != -1) {
                "${block.color.name}[${block.id}]: ${"%.1f".format(block.angle)} deg"
            } else {
                "${block.color.name}: ${"%.1f".format(block.angle)} deg"
            }
            val line2 = "stdHSV=(${"%.1f".format(block.colorStd.first)}, ${"%.1f".format(block.colorStd.second)}, ${
                "%.1f".format(block.colorStd.third)
            })"
            val line3 = "avgHSV=(${"%.1f".format(block.colorMean.first)}, ${"%.1f".format(block.colorMean.second)}, ${
                "%.1f".format(block.colorMean.third)
            })"
            val lines = listOf(line1, line2, line3)
            for ((i, line) in lines.withIndex()) {
                val pt = Point(x0.toDouble(), (y0 + i * 15).toDouble())
                Imgproc.putText(
                    output,
                    line,
                    pt,
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    Scalar(255.0, 255.0, 255.0),
                    1
                )
            }
        }

        dbgData["blocks"] = output

        return output
    }
}
