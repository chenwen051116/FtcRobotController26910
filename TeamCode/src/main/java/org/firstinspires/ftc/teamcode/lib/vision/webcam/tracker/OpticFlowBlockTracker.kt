package org.webcam_visual.tracker

import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.imgproc.Imgproc
import org.webcam_visual.common.Block
import org.webcam_visual.common.FrameCtx
import java.util.*

/**
 * OpticFlowBlockTracker uses optical flow to track blocks across frames.
 *
 * For the first frame, it assigns incremental IDs. For subsequent frames,
 * it samples a number of points inside each block and, using the motion vectors,
 * maps these sample points to the previous frame. If a sufficient fraction
 * (configurable) of the sample points fall inside a previous block (with the same color),
 * then the current block is assigned that block’s ID.
 */
class OpticFlowBlockTracker(val config: Cfg = Cfg()) : BlockTracker {

    // Next available block ID.
    private var nextId = 0
        get() = field.also {
            if (field >= 50) field = 0 else
            field++
        }

    /**
     * Configuration parameters for the block tracker.
     *
     * @property sampleCount Number of sample points to generate inside each block.
     * @property matchThreshold Fraction (0.0–1.0) of sample points that must match a previous block.
     * @property sampleAreaFactor Multiplier (<1.0) to shrink the block's size when sampling points.
     */
    data class Cfg(
        val sampleCount: Int = 35,
        val matchThreshold: Double = 0.4,
        val sampleAreaFactor: Double = .7
    )

    /**
     * Tracks blocks between the previous and current frame.
     *
     * @param ctx The current frame context; its prevBlocks field holds blocks from the previous frame.
     * @return List of blocks for the current frame, with IDs assigned.
     */
    override fun trackBlocks(ctx: FrameCtx): FrameCtx {
        // For the first frame, simply assign incremental IDs.
        if (ctx.prevBlocks == null) {
            val tracked = ctx.curBlocks!!.map { block ->
                block.copy(id = nextId)
            }
            ctx.prevBlocks = tracked
            return ctx.copy(curBlocks = tracked)
        } else {
            // Get the optical flow computed in the FrameCtx.
            val flow = ctx.ensureOpticFlow()
            val tracked = ctx.curBlocks!!.map { currBlock ->
                // Generate sample points inside the current block.
                val samples = samplePointsInBlock(currBlock, config.sampleCount, config.sampleAreaFactor)
                var bestCandidate: Block? = null
                var bestFraction = 0.0
                // Look for candidate blocks in the previous frame that have the same color.
                for (prevBlock in ctx.prevBlocks!!) {
                    if (prevBlock.color.name == currBlock.color.name) {
                        var count = 0
                        // For each sample, compute its corresponding location in the previous frame.
                        for (pt in samples) {
                            // Get the flow vector at the sample point.
                            val flowVec = getFlowAt(flow, pt)
                            if (flowVec != null) {
                                // Map the sample point using the flow.
                                val ptPrev = Point(pt.x + flowVec.x, pt.y + flowVec.y)
                                // Count the sample if it falls inside the candidate previous block.
                                if (isPointInsideBlock(ptPrev, prevBlock)) {
                                    count++
                                }
                            }
                        }
                        val fraction = count.toDouble() / samples.size
                        println("Fraction: $fraction")
                        if (fraction >= config.matchThreshold && fraction > bestFraction) {
                            bestCandidate = prevBlock
                            bestFraction = fraction
                        }
                    }
                }
                if (bestCandidate != null) {
                    // Use the same ID as the best candidate.
                    currBlock.copy(id = bestCandidate.id)
                } else {
                    // Assign a new id if no candidate meets the threshold.
                    currBlock.copy(id = nextId)
                }
            }
            ctx.prevBlocks = tracked
            return ctx.copy(curBlocks = tracked)
        }
    }

    /**
     * Samples [sampleCount] points uniformly inside the current block.
     * The block's size is first shrunk by [shrinkFactor] so that the samples are more likely to lie inside.
     */
    private fun samplePointsInBlock(block: Block, sampleCount: Int, shrinkFactor: Double): List<Point> {
        val samples = mutableListOf<Point>()
        val rand = Random()
        // Use the block's size and shrink factor to define sampling ranges.
        val halfWidth = block.size.first / 2 * shrinkFactor
        val halfHeight = block.size.second / 2 * shrinkFactor
        // Block center and rotation (angle in degrees converted to radians).
        val cx = block.center.first
        val cy = block.center.second
        val angleRad = Math.toRadians(block.angle.toDouble())
        for (i in 0 until sampleCount) {
            // Generate random offsets within [-halfWidth, halfWidth] and [-halfHeight, halfHeight].
            val dx = (rand.nextDouble() * 2 - 1) * halfWidth
            val dy = (rand.nextDouble() * 2 - 1) * halfHeight
            // Rotate the offsets by the block's orientation.
            val rx = dx * Math.cos(angleRad) - dy * Math.sin(angleRad)
            val ry = dx * Math.sin(angleRad) + dy * Math.cos(angleRad)
            samples.add(Point(cx + rx, cy + ry))
        }
        return samples
    }

    /**
     * Retrieves the optical flow vector at the given point [pt] from the [flow] matrix.
     * Returns null if the point is outside the valid range.
     */
    private fun getFlowAt(flow: Mat, pt: Point): Point? {
        val x = pt.x.toInt()
        val y = pt.y.toInt()
        if (x < 0 || x >= flow.cols() || y < 0 || y >= flow.rows()) return null
        val flowVec = flow.get(y, x)  // Expected to return a DoubleArray of size 2.
        return if (flowVec != null && flowVec.size >= 2) Point(flowVec[0], flowVec[1]) else null
    }

    /**
     * Determines whether the given point [pt] lies inside the contour of [block].
     */
    private fun isPointInsideBlock(pt: Point, block: Block): Boolean {
        val contour2f = org.opencv.core.MatOfPoint2f(*block.contour.toArray())
        return Imgproc.pointPolygonTest(contour2f, pt, false) >= 0
    }
}
