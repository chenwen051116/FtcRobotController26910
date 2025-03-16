package org.webcam_visual.detectors

import org.webcam_visual.common.ImgDebuggable
import org.webcam_visual.common.Block
import org.webcam_visual.common.FrameCtx

interface BlockDetector : ImgDebuggable {
    fun detectBlocks(ctx: FrameCtx): FrameCtx
}