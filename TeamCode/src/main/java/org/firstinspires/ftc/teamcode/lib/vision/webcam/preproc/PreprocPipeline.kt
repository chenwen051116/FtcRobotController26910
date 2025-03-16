package org.webcam_visual.preproc

import org.opencv.core.Mat
import org.webcam_visual.common.DefaultImgDebuggable
import org.webcam_visual.common.ImgDebuggable
import org.webcam_visual.common.FrameCtx

class PreprocPipeline(
    vararg steps: PreprocStep
) : DefaultImgDebuggable(){
    val steps: MutableList<PreprocStep> = steps.toMutableList()
    // Add a step dynamically.
    init {
        setDbgOption("original", true)
        for (step in steps) {
            dbgChildren.add(step)
        }
    }

    fun addStep(step: PreprocStep) {
        steps.add(step)
        dbgChildren.add(step)
    }

    // Process an image through all steps sequentially.
    fun process(ctx: FrameCtx): FrameCtx {
        if (!isDebugDisabled) {
            dbgData["original"] = ctx.frame!!
        }
        var result = ctx.copy()
        for ((idx, step) in steps.withIndex()) {
            result = step.process(result)
        }
        return result
    }
}