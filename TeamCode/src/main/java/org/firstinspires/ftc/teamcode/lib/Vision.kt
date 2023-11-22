package org.firstinspires.ftc.teamcode.lib

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.teamcode.lib.math.Vector
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import java.lang.Thread.sleep
import kotlin.concurrent.thread

class Vision {
    lateinit var instance: AprilTagStream
    lateinit var portal: VisionPortal
    lateinit var lastDetection: AprilTagDetection
    private lateinit var fieldTags: Map<Int, Vector>

    val tagRelativePosition = Vector(0.0, 0.0)
    var fieldRelativePosition = Vector(0.0, 0.0)
    var heading = 0.0

    class LensIntrinsics(val fx: Double = 0.0, val fy: Double = 0.0, val cx: Double = 0.0, val cy: Double = 0.0)

    fun updatePosition(detection: AprilTagDetection) {
        if(detection.metadata == null) return

        tagRelativePosition.x = detection.ftcPose.x
        tagRelativePosition.y = detection.ftcPose.y
        tagRelativePosition.z = detection.ftcPose.y
        this.heading = detection.ftcPose.bearing

        val fieldTag = this.fieldTags[detection.id]?.clone() ?: Vector(0.0, 0.0)
        fieldTag.sub(tagRelativePosition)
        this.fieldRelativePosition = fieldTag
    }

    private fun init(lensIntrinsics: LensIntrinsics, decimation: Float = 2f, fieldTags: Map<Int, Vector>) {
        this.fieldTags = fieldTags

        instance = AprilTagStream.Builder().setLensIntrinsics(
                lensIntrinsics.fx,
                lensIntrinsics.fy,
                lensIntrinsics.cx,
                lensIntrinsics.cy
        ).build()

        instance.setDecimation(decimation)
    }

    fun init(lensIntrinsics: LensIntrinsics, decimation: Float = 2f, webcam: WebcamName, fieldTags: Map<Int, Vector>) {
        this.init(lensIntrinsics, decimation, fieldTags)

        this.portal = VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(this.instance)
                .build()
    }

    fun init(lensIntrinsics: LensIntrinsics, decimation: Float = 2f, cameraDirection: BuiltinCameraDirection, fieldTags: Map<Int, Vector>) {
        this.init(lensIntrinsics, decimation, fieldTags)

        this.portal = VisionPortal.Builder()
                .setCamera(cameraDirection)
                .addProcessor(this.instance)
                .build()
    }

    val detections: ArrayList<AprilTagDetection>
        get() {
            val detections = this.instance.detections
            if(detections.size > 0) {
                this.lastDetection = detections[0]
                this.updatePosition(detections[0])
            }

            return detections
        }
}