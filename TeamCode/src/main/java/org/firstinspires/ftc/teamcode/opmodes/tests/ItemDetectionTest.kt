package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.OpMode
import org.firstinspires.ftc.teamcode.lib.vision.detection.ItemDetector
import org.opencv.core.Scalar
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam


@Config
@Autonomous(name = "Item Detection Test", group = "FTC23_TEST_VISION")
class ItemDetectionTest : OpMode() {
    val width: Int = 640
    val height: Int = 480
    val detector = ItemDetector()


    companion object {
        @JvmField
        var lowHSV = Scalar(0.0, 180.0, 160.0)
        @JvmField
        var highHSV = Scalar(5.0, 230.0, 240.0)
        @JvmField
        var threshold = 0.15
    }

    override fun setup() {
        val webcamName:WebcamName = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        val monitorViewIdParent = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val webcam: OpenCvWebcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, monitorViewIdParent)
        detector.init(width, height, lowHSV, highHSV, threshold, telemetry)
        webcam.setPipeline(detector)
        webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT)
                FtcDashboard.getInstance().startCameraStream(webcam, 30.0)
            }

            override fun onError(errorCode: Int) {}
        })
    }

    override fun run() {
        while (opModeIsActive()) {

        }
    }

}