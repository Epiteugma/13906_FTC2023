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
import kotlin.properties.Delegates

class ItemDetector : OpenCvPipeline() {
    private var THRESHOLD: Double = 0.0
    lateinit var location: Location
    private lateinit var mat: Mat
    private lateinit var lowHSV: Scalar
    private lateinit var highHSV: Scalar
    private var width by Delegates.notNull<Int>()
    private var height by Delegates.notNull<Int>()
    private var LEFT_ROI = Rect(Point(0.0, 0.0), Point(0.0, 0.0))
    private var RIGHT_ROI = Rect(Point(0.0, 0.0), Point(0.0, 0.0))
    private var CENTER_ROI = Rect(Point(0.0, 0.0), Point(0.0, 0.0))
    private var telemetry: Telemetry? = null


    enum class Location {
        NONE,
        LEFT,
        RIGHT,
        CENTER
    }
    fun init(width:Int, height:Int, lowHsv: Scalar, highHsv: Scalar, threshold: Double, telemetry: Telemetry) {
        this.width = width
        this.height = height
        this.LEFT_ROI = Rect(Point(0.0, height / 2.0), Point(width / 3.0, height.toDouble()))
        this.RIGHT_ROI = Rect(Point(width * 2.0 / 3.0, height / 2.0), Point(width.toDouble(), height.toDouble()))
        this.CENTER_ROI = Rect(Point(width / 3.0, height / 2.0), Point(width * 2.0 / 3.0, height.toDouble()))
        this.mat = Mat()
        this.THRESHOLD = threshold
        this.lowHSV = lowHsv
        this.highHSV = highHsv
        this.location = Location.NONE
        this.telemetry = telemetry

    }

    override fun processFrame(input: Mat): Mat {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV)
        Core.inRange(mat, lowHSV, highHSV, mat)
        val left = mat.submat(LEFT_ROI)
        val right = mat.submat(RIGHT_ROI)
        val center = mat.submat(CENTER_ROI)
        val leftValue = Core.sumElems(left).`val`[0] / LEFT_ROI.area() / 255
        val rightValue = Core.sumElems(right).`val`[0] / RIGHT_ROI.area() / 255
        val centerValue = Core.sumElems(center).`val`[0] / CENTER_ROI.area() / 255
        this.telemetry?.addData("Left", leftValue)
        this.telemetry?.addData("Right", rightValue)
        this.telemetry?.addData("Center", centerValue)
        this.telemetry?.update()
        left.release()
        right.release()
        center.release()
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB)
        val values = doubleArrayOf(leftValue, centerValue, rightValue)
        Arrays.sort(values)
        if (values[2] > THRESHOLD) {
        if (leftValue == values[2]) location =
            Location.LEFT else if (rightValue == values[2]) location =
            Location.RIGHT else if (centerValue == values[2]) location = Location.CENTER
        }
        val found = Scalar(0.0, 255.0, 0.0)
        val notFound = Scalar(255.0, 0.0, 0.0)
        Imgproc.rectangle(mat, LEFT_ROI, if (location == Location.LEFT) found else notFound)
        Imgproc.rectangle(mat, RIGHT_ROI, if (location == Location.RIGHT) found else notFound)
        Imgproc.rectangle(mat, CENTER_ROI, if (location == Location.CENTER) found else notFound)
        return mat
    }
}