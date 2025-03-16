package org.webcam_visual.preproc

import org.opencv.core.Mat
import org.webcam_visual.common.DefaultImgDebuggable
import org.webcam_visual.common.FrameCtx

abstract class PreprocStep(val stepName: String, val initDebug: Boolean = false) : DefaultImgDebuggable() {
    init {
        // Optionally preconfigure the debug option "output" for this step.
        setDbgOption("output", initDebug)
    }

    fun process(ctx: FrameCtx): FrameCtx{
        val result = _process(ctx)
        if (isDbgOptionEnabled("output")) addDbgEntry("output", result.clone())
        return ctx.copy(frame = result)
    }
    abstract protected fun _process(ctx: FrameCtx): Mat
}