package org.webcam_visual.preproc

import org.opencv.core.*
import org.webcam_visual.common.FrameCtx
import org.webcam_visual.utils.mat.createColorMappedImageWithLegend
import org.webcam_visual.utils.mat.drawFlowArrows
import org.webcam_visual.utils.mat.gt

/**
 * TemporalDenoiserStep uses DIS optical flow and temporal blending.
 * It now accepts a FrameCtx object that holds intermediate results for the current frame.
 */
class TemporalDenoiserStep(
    private var alpha: Double = 0.8,         // Weight for current frame in blending.
    private var threshold: Double = 30.0,      // Threshold for rejecting motion compensation.
    private val flowGridStep: Int = 20         // Grid step size for flow visualization.
) : PreprocStep("temporal_denoise") {
    init{
        setDbgOptions(listOf("flow", "warp", "diff", "mask", "output"), initDebug)
    }
    // Note: In a production system, the prevFrame should be maintained across frames.
    // Here we assume the FrameCtx is passed in with a valid previous frame.

    /**
     * Processes the current FrameCtx by computing intermediate results on demand and blending the frame.
     * The intermediate results (optical flow, motion-compensated frame, and color difference) are computed
     * only if they are not already stored in the FrameCtx.
     *
     * @param ctx The FrameCtx containing the current frame and previous frame.
     * @return The blended frame after temporal denoising.
     */
    override fun _process(ctx: FrameCtx): Mat {
        if (ctx.prevFrame == null) {
            return ctx.frame!!.clone()
        }

        if (isDbgOptionEnabled("flow")){
            val flowVis = drawFlowArrows(ctx.ensureOpticFlow(), flowGridStep)
            addDbgEntry("flow", flowVis)
        }

        // If debug option "diff" is enabled, generate a color-mapped visualization.
        if (isDbgOptionEnabled("diff")) {
            val (colorDiff, colorBar) = createColorMappedImageWithLegend(ctx.ensureColorDisAfterMvec())
            // Overlay the color bar in the top-right corner.
            val roi = colorDiff.submat(
                0, colorBar.rows(),
                colorDiff.cols() - colorBar.cols(), colorDiff.cols()
            )
            colorBar.copyTo(roi)
            addDbgEntry("diff", colorDiff)
        }

        // Build a mask where the computed distance exceeds the threshold.
        val mask = ctx.ensureColorDisAfterMvec() gt Scalar(threshold)
        if (isDbgOptionEnabled("mask")) {
            addDbgEntry("mask", mask.clone())
        }
        // Blend the current frame with the warped previous frame.
        val blended = Mat()
        Core.addWeighted(ctx.frame!!, alpha, ctx.ensureWarpedByMvec(), 1.0 - alpha, 0.0, blended)
        // Override pixels in the blended image with the current frame where mask is set.
        ctx.frame!!.copyTo(blended, mask)
        if (isDbgOptionEnabled("output"))
            addDbgEntry("output", blended.clone())

        // In a real pipeline, update the persistent previous frame here.
        return blended
    }
}
