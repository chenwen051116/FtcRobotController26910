package org.webcam_visual.pipeline

import org.opencv.core.Mat
import org.webcam_visual.common.DefaultImgDebuggable
import org.webcam_visual.common.FrameCtx
import org.webcam_visual.detectors.BlockDetector
import org.webcam_visual.preproc.PreprocPipeline
import org.webcam_visual.tracker.BlockTracker
import org.webcam_visual.visualizer.BlockVisualizer

class RobotVisionPipeline(
    val preproc: PreprocPipeline?,
    val detector: BlockDetector,
    val tracker: BlockTracker?,
    val visualizer: BlockVisualizer?,
) : DefaultImgDebuggable() {
    init {
        preproc?.let { addDbgChild(it) }
        addDbgChild(detector)
        visualizer?.let { addDbgChild(it) }
    }

    private var frameCtx = FrameCtx()

    fun updateFrame(frame: Mat) : FrameCtx {
        frameCtx.updateFrame(frame)
        frameCtx = preproc?.process(frameCtx) ?: frameCtx
        frameCtx = detector.detectBlocks(frameCtx)
        tracker?.let { frameCtx = it.trackBlocks(frameCtx) }
        visualizer?.visualizeBlocks(frameCtx)
        frameCtx.curBlocks?.forEach{
            it.calcRelative(frame.width(), frame.height())
        }
        return frameCtx
    }
}