package org.webcam_visual.common

import Block
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.opencv.video.DISOpticalFlow
import org.webcam_visual.utils.mat.plus
import org.webcam_visual.utils.mat.times

/**
 * FrameCtx holds intermediate results for a frame.
 *
 * @property frame The current frame (BGR Mat).
 * @property prevFrame The previous frame (BGR Mat).
 * @property opticFlow Optical flow (CV_32FC2) computed from prevFrame and frame.
 * @property warpedByMvec The previous frame warped using the optical flow.
 * @property colorDisAfterMvec The per-pixel color difference (CV_32FC1) between frame and warpedByMvec.
 */
data class FrameCtx(
    var frame: Mat,
    var prevFrame: Mat? = null,
    var opticFlow: Mat? = null,
    var warpedByMvec: Mat? = null,
    var colorDisAfterMvec: Mat? = null,
    var curBlocks : List<Block>? = null,
    var prevBlocks : List<Block>? = null
) {
    /**
     * Ensures that optical flow is computed.
     * If opticFlow is null, computes it from prevFrame and frame.
     * (Assumes both frames are in BGR; converts them to grayscale before computation.)
     */
    fun ensureOpticFlow(): Mat {
        assert(prevFrame != null) { "prevFrame must be set before computing optical flow." }
        if (opticFlow == null) {
            val currGray = Mat()
            val prevGray = Mat()
            Imgproc.cvtColor(frame, currGray, Imgproc.COLOR_BGR2GRAY)
            Imgproc.cvtColor(prevFrame, prevGray, Imgproc.COLOR_BGR2GRAY)
            val flow = Mat(currGray.size(), CvType.CV_32FC2)
            // Compute optical flow using DISOpticalFlow (PRESET_ULTRAFAST)
            DISOpticalFlow.create(DISOpticalFlow.PRESET_ULTRAFAST).calc(prevGray, currGray, flow)
            opticFlow = flow
        }
        return opticFlow!!
    }

    /**
     * Ensures that the warped frame (using the motion vector/optical flow) is computed.
     * If warpedByMvec is null, computes it by remapping prevFrame using the optical flow.
     */
    fun ensureWarpedByMvec(): Mat {
        if (warpedByMvec == null) {
            val flow = ensureOpticFlow()
            val size = frame.size()
            // Create coordinate grids
            val gridX = Mat(size, CvType.CV_32FC1)
            val gridY = Mat(size, CvType.CV_32FC1)
            for (row in 0 until size.height.toInt()) {
                for (col in 0 until size.width.toInt()) {
                    gridX.put(row, col, col.toDouble())
                    gridY.put(row, col, row.toDouble())
                }
            }
            val flowChannels = ArrayList<Mat>(2)
            Core.split(flow, flowChannels)  // flowChannels[0]=dx, flowChannels[1]=dy
            // Using our operator extensions: add operator (+) replaces Core.add.
            val flowMapX = flowChannels[0] + gridX
            val flowMapY = flowChannels[1] + gridY
            val warped = Mat(frame.size(), frame.type())
            // Remap the previous frame using the computed flow maps.
            Imgproc.remap(prevFrame, warped, flowMapX, flowMapY, Imgproc.INTER_LINEAR, Core.BORDER_CONSTANT, Scalar(0.0, 0.0, 0.0))
            warpedByMvec = warped
        }
        return warpedByMvec!!
    }

    /**
     * Ensures that the color difference after motion compensation is computed.
     * Computes the per-pixel distance between frame and warpedByMvec.
     */
    fun ensureColorDisAfterMvec(): Mat {
        if (colorDisAfterMvec == null) {
            val warped = ensureWarpedByMvec()
            val diff = Mat()
            Core.absdiff(frame, warped, diff)
            val diffFloat = Mat()
            diff.convertTo(diffFloat, CvType.CV_32F)
            // Square the differences using our multiplication operator (*).
            val diffSquared = diffFloat * diffFloat
            val diffChannels = ArrayList<Mat>()
            Core.split(diffSquared, diffChannels)
            // Sum the channels using our plus operator.
            val sumOfSquares = diffChannels[0] + diffChannels[1] + diffChannels[2]
            val distMat = Mat()
            Core.sqrt(sumOfSquares, distMat)
            colorDisAfterMvec = distMat
        }
        return colorDisAfterMvec!!
    }

    fun updateFrame(nextFrame: Mat) {
        prevFrame = frame
        frame = nextFrame
        opticFlow = null
        warpedByMvec = null
        colorDisAfterMvec = null
        prevBlocks = curBlocks
        curBlocks = null
    }

}
