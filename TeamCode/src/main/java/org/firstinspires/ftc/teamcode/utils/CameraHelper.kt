import android.graphics.Bitmap
import android.graphics.ImageFormat
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.android.util.Size
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.hardware.camera.*
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import org.opencv.android.Utils
import org.opencv.core.Mat
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit

/**
 * Helper class to initialize the camera and retrieve frames as OpenCV Mats.
 * Ensure that OpenCV is properly loaded in your project.
 */
class CameraHelper(private val hardwareMap: com.qualcomm.robotcore.hardware.HardwareMap) {

    companion object {
        private const val TAG = "CameraHelper"
        private const val PERMISSION_TIMEOUT_SECONDS = Integer.MAX_VALUE.toLong()
    }

    // Camera and session objects
    private val cameraManager: CameraManager = ClassFactory.getInstance().cameraManager
    private val cameraName: WebcamName = hardwareMap.get(WebcamName::class.java, "Webcam")
    private var camera: Camera? = null
    private var cameraCaptureSession: CameraCaptureSession? = null

    // The frame queue that holds Bitmap frames as they become available.
    private lateinit var frameQueue: EvictingBlockingQueue<Bitmap>

    // Handler for asynchronous callbacks
    private val callbackHandler = CallbackLooper.getDefault().handler

    init {
        initializeFrameQueue(2)
        openCamera()
        startCamera()
    }

    /**
     * Initializes the frame queue with the specified capacity.
     * Frames that are not processed quickly are automatically recycled.
     */
    private fun initializeFrameQueue(capacity: Int) {
        frameQueue = EvictingBlockingQueue(ArrayBlockingQueue(capacity))
        frameQueue.setEvictAction { frame ->
            // Recycle the bitmap if it is discarded without processing.
            frame.recycle()
        }
    }

    /**
     * Opens the camera by requesting permission and opening the camera device.
     */
    private fun openCamera() {
        if (camera != null) return  // Already opened
        val deadline = Deadline(PERMISSION_TIMEOUT_SECONDS, TimeUnit.SECONDS)
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null)
        if (camera == null) {
            error("Camera not found or permission not granted: $cameraName")
        }
    }

    /**
     * Starts the camera capture session.
     *
     * This sets up a capture session for the YUY2 image format, starts streaming frames,
     * and places each copied Bitmap frame into the frame queue.
     */
    private fun startCamera() {
        if (cameraCaptureSession != null) return  // Already started

        val imageFormat = ImageFormat.YUY2

        // Verify that the image format is supported by the camera.
        val cameraCharacteristics: CameraCharacteristics = cameraName.cameraCharacteristics
        if (!contains(cameraCharacteristics.androidFormats, imageFormat)) {
            error("Image format not supported")
            return
        }

        // Get the default size and maximum frame rate for the chosen format.
        val size: Size = cameraCharacteristics.getDefaultSize(imageFormat)
        val fps: Int = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size)

        // Synchronizer to wait until the asynchronous setup completes.
        val synchronizer = ContinuationSynchronizer<CameraCaptureSession>()
        try {
            camera?.createCaptureSession(
                Continuation.create(callbackHandler, object : CameraCaptureSession.StateCallbackDefault() {
                    override fun onConfigured(session: CameraCaptureSession) {
                        try {
                            // Create a capture request for the specified format, size, and frame rate.
                            val captureRequest = camera!!.createCaptureRequest(imageFormat, size, fps)
                            session.startCapture(
                                captureRequest,
                                object : CameraCaptureSession.CaptureCallback {
                                    override fun onNewFrame(
                                        session: CameraCaptureSession,
                                        request: CameraCaptureRequest,
                                        cameraFrame: CameraFrame
                                    ) {
                                        // Copy the frame into a Bitmap.
                                        val bmp: Bitmap = captureRequest.createEmptyBitmap()
                                        cameraFrame.copyToBitmap(bmp)
                                        frameQueue.offer(bmp)
                                    }
                                },
                                Continuation.create(callbackHandler, object : CameraCaptureSession.StatusCallback {
                                    override fun onCaptureSequenceCompleted(
                                        session: CameraCaptureSession,
                                        cameraCaptureSequenceId: CameraCaptureSequenceId,
                                        lastFrameNumber: Long
                                    ) {
                                        RobotLog.ii(TAG, "Capture sequence $cameraCaptureSequenceId completed: lastFrame=$lastFrameNumber")
                                    }
                                })
                            )
                            synchronizer.finish(session)
                        } catch (e: Exception) {
                            RobotLog.ee(TAG, e, "Exception starting capture")
                            error("Exception starting capture")
                            session.close()
                            synchronizer.finish(null)
                        }
                    }
                })
            )
        } catch (e: Exception) {
            RobotLog.ee(TAG, e, "Exception starting camera")
            error("Exception starting camera")
            synchronizer.finish(null)
        }

        // Wait for the asynchronous configuration to finish.
        try {
            synchronizer.await()
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        }
        cameraCaptureSession = synchronizer.value
    }

    /**
     * Returns the current frame from the camera as an OpenCV Mat.
     *
     * @param timeoutMs the maximum time to wait for a frame, in milliseconds (default 1000ms).
     * @return the current frame as a Mat, or null if no frame is available within the timeout.
     */
    fun getCurrentFrameMat(timeoutMs: Long = 1000): Mat? {
        val bmp = try {
            frameQueue.poll(timeoutMs, TimeUnit.MILLISECONDS)
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
            return null
        }
        bmp ?: return null

        val mat = Mat()
        // Convert the Bitmap to an OpenCV Mat.
        Utils.bitmapToMat(bmp, mat)
        // Recycle the Bitmap now that we've converted it.
        bmp.recycle()
        return mat
    }

    /**
     * Closes the camera capture session and the camera.
     */
    fun closeCamera() {
        cameraCaptureSession?.let {
            it.stopCapture()
            it.close()
        }
        cameraCaptureSession = null
        camera?.close()
        camera = null
    }

    /**
     * Helper function to log errors.
     */
    private fun error(msg: String) {
        RobotLog.ee(TAG, msg)
    }

    /**
     * Checks whether the given value is contained in the array.
     */
    private fun contains(array: IntArray, value: Int): Boolean {
        return array.any { it == value }
    }
}
