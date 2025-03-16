package org.yourorg.yourpackage

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.lib.shTelemetry
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import org.opencv.core.Mat

class CameraHelper(private val hardwareMap: HardwareMap) {

    private val camera: OpenCvCamera
    private val pipeline = FrameCapturePipeline()

    init {
        // Retrieve the webcam from the hardwareMap (make sure the configuration name matches)
        val webcamName = hardwareMap.get(WebcamName::class.java, "Webcam")
        // Create the OpenCvCamera instance using the factory
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName)!!
        // Set our pipeline that captures the latest frame
        camera.setPipeline(pipeline)
        // Open the camera asynchronously and start streaming once opened
//        camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
//            override fun onOpened() {
//                camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);
//
//                shTelemetry!!.addLine("in onOpened")
//                camera.startStreaming( 320, 240, OpenCvCameraRotation.UPRIGHT)
//                shTelemetry!!.addLine("after started streaming")
//                shTelemetry!!.update()
//            }
//
//            override fun onError(errorCode: Int) {
//                // Handle error if needed
//                throw Exception("error in openCameraDeviceAsync")
//            }
//        })
        camera.openCameraDevice()
        shTelemetry!!.addLine("camera opened");
        shTelemetry!!.update()
        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);

        camera.startStreaming( 640, 480, OpenCvCameraRotation.UPRIGHT)
    }

    /**
     * Returns the most recent frame as an OpenCV Mat.
     * @return the newest frame Mat or null if no frame is available.
     */
    @Synchronized
    fun getLatestFrame(): Mat? {
        shTelemetry!!.addLine("in getLatestFrame")
        shTelemetry!!.update()
        return pipeline.latestFrame
    }

    /**
     * Stops the camera streaming and releases the camera.
     */
    fun stop() {
        camera.stopStreaming()
        camera.closeCameraDevice()
    }

    /**
     * OpenCvPipeline implementation that stores a clone of the most recent frame.
     */
    private class FrameCapturePipeline : OpenCvPipeline() {
        @Volatile
        var latestFrame: Mat? = null

        override fun processFrame(input: Mat): Mat {
            // Clone the input Mat and store it as the latest frame.
            shTelemetry!!.addLine("process frame called")
            shTelemetry!!.update()
            throw Exception("process frame called")
            latestFrame = input
            // Return the unmodified frame for display.
            return input
        }
    }
}
