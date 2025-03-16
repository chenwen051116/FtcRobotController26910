package org.firstinspires.ftc.teamcode.lib.vision.detector

import org.firstinspires.ftc.teamcode.lib.vision.common.ImgDebuggable
import org.firstinspires.ftc.teamcode.lib.vision.common.Block
import org.firstinspires.ftc.teamcode.lib.vision.common.FrameCtx

interface BlockDetector : ImgDebuggable {
    fun detectBlocks(ctx: FrameCtx): FrameCtx
}