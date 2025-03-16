package org.yourorg.yourpackage

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.lib.shTelemetry
import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import org.openftc.easyopencv.OpenCvWebcam

class CameraHelper(private val hardwareMap: HardwareMap) {

    val camera: OpenCvWebcam
    val pipeline = FrameCapturePipeline()
    init {
        // Retrieve the webcam from the hardwareMap (make sure the configuration name matches)

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        val webcamName = hardwareMap.get(WebcamName::class.java, "Webcam")
        // Create the OpenCvCamera instance using the factory
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)!!
        // Set our pipeline that captures the latest frame
        camera.setPipeline(pipeline)
        camera.setMillisecondsPermissionTimeout(5000)
        // Open the camera asynchronously and start streaming once opened
        camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                shTelemetry!!.addLine("in onOpened")
                camera.startStreaming( 320, 240, OpenCvCameraRotation.UPRIGHT)
                shTelemetry!!.addLine("after started streaming")
                shTelemetry!!.update()
            }

            override fun onError(errorCode: Int) {
                // Handle error if needed
                throw Exception("error in openCameraDeviceAsync")
            }
        })

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
    class FrameCapturePipeline : OpenCvPipeline() {
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
