package org.firstinspires.ftc.teamcode.lib.vision.detection

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import java.util.Arrays

class ItemDetector(private val width: Int, private val height: Int, private val lowHSV: Scalar, private val highHSV: Scalar, private val telemetry: Telemetry) : OpenCvPipeline() {
    var location = Location.NONE
    private var mat: Mat = Mat()
    private val margin = 3.0

    // top left corner is (0, 0)
    // Left is slightly lower as it is closer
    private val RIGHT_ROI = Rect(
        Point(0.0 + margin, height * 0.4),
        Point(width / 3.0 - margin, height * 0.8)
    )

    // center must be smaller as it is the furthest away
    private val CENTER_ROI = Rect(
        Point(width / 3.0, height * 0.25),
        Point(width * 2.0 / 3.0, height * 0.6)
    )
    private val LEFT_ROI = Rect(
        Point(width * 2.0 / 3.0 + margin, height * 0.25),
        Point(width - margin, height * 0.75)
    )

    enum class Location(val value: Double) {
        NONE(0.0),
        LEFT(0.15),
        CENTER(0.1),
        RIGHT(0.15)

    }

    override fun processFrame(input: Mat): Mat {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV)
        Core.inRange(mat, lowHSV, highHSV, mat)
        val left = mat.submat(LEFT_ROI)
        val center = mat.submat(CENTER_ROI)
        val right = mat.submat(RIGHT_ROI)
        val leftValue = Core.sumElems(left).`val`[0] / LEFT_ROI.area() / 255
        val centerValue = Core.sumElems(center).`val`[0] / CENTER_ROI.area() / 255
        val rightValue = Core.sumElems(right).`val`[0] / RIGHT_ROI.area() / 255
        telemetry.addData("Left", leftValue)
        telemetry.addData("Center", centerValue)
        telemetry.addData("Right", rightValue)
        left.release()
        right.release()
        center.release()
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB)
        val values = doubleArrayOf(leftValue, centerValue, rightValue)
        Arrays.sort(values)
        // check if the value is above its threshold
        if (leftValue == values[2] && leftValue > Location.LEFT.value) location =
            Location.LEFT else if (centerValue == values[2] && centerValue > Location.CENTER.value) location =
            Location.CENTER else if (rightValue == values[2] && rightValue > Location.RIGHT.value) location =
            Location.RIGHT
        telemetry.addData("Location", location)
        telemetry.update()
        val found = Scalar(0.0, 255.0, 0.0)
        val notFound = Scalar(255.0, 0.0, 0.0)
        Imgproc.rectangle(mat, LEFT_ROI, if (location == Location.LEFT) found else notFound)
        Imgproc.rectangle(mat, RIGHT_ROI, if (location == Location.RIGHT) found else notFound)
        Imgproc.rectangle(mat, CENTER_ROI, if (location == Location.CENTER) found else notFound)
        return mat
    }
}
