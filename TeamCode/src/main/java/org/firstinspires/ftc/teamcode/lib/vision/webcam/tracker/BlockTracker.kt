package org.webcam_visual.tracker

import org.webcam_visual.common.FrameCtx

interface BlockTracker {
    fun trackBlocks(ctx: FrameCtx): FrameCtx
}