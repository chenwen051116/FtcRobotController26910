package org.webcam_visual.pipeline

import org.opencv.core.Mat
import org.webcam_visual.common.DefaultImgDebuggable
import org.webcam_visual.common.FrameCtx
import org.webcam_visual.detectors.BlockDetector
import org.webcam_visual.preproc.PreprocPipeline
import org.webcam_visual.tracker.BlockTracker
import org.webcam_visual.visualizer.BlockVisualizer

class RobotVisionPipeline(
    val preproc: PreprocPipeline? = null,
    val detector: BlockDetector,
    val tracker: BlockTracker? = null,
    val visualizer: BlockVisualizer? = null,
) : DefaultImgDebuggable() {
    init {
        preproc?.let { addDbgChild(it) }
        addDbgChild(detector)
        visualizer?.let { addDbgChild(it) }
    }

    private var frameCtx = FrameCtx(Mat())

    fun updateFrame(frame: Mat) : FrameCtx {
        if (frameCtx.prevFrame == null) {
            frameCtx = frameCtx.copy(prevFrame=frame)
        }
        frameCtx = frameCtx.copy(frame=frame)
        preproc?.let { frameCtx = it.process(frameCtx) }
        frameCtx = detector.detectBlocks(frameCtx)
        tracker?.let { frameCtx = it.trackBlocks(frameCtx) }
        visualizer?.visualizeBlocks(frameCtx)
        frameCtx.updateFrame(frame)
        frameCtx.curBlocks?.forEach{
            it.calcRelative(frame.width(), frame.height())
        }
        return frameCtx
    }
}