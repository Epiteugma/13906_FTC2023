package org.firstinspires.ftc.teamcode.lib.vision.detection

import com.amarcolini.joos.dashboard.JoosConfig
import org.firstinspires.ftc.teamcode.Color
import org.opencv.core.Scalar

class Prop(private val color: Color) {
    val lowHSV: Scalar
    val highHSV: Scalar
    val threshold: Double

    @JoosConfig
    private val blueLowHSV = Scalar(80.0, 100.0, 240.0) // (0-180, 0-255, 0-255)
    @JoosConfig
    private val blueHighHSV = Scalar(180.0, 255.0, 255.0) // (0-180, 0-255, 0-255)
    @JoosConfig
    private val bothLowHSV = Scalar(80.0, 100.0, 200.0) // (0-180, 0-255, 0-255)
    @JoosConfig
    private val bothHighHSV = Scalar(180.0, 255.0, 255.0) // (0-180, 0-255, 0-255)
    @JoosConfig
    private val redLowHSV = Scalar(155.0, 0.0, 150.0) // (0-180, 0-255, 0-255)
    @JoosConfig
    private val redHighHSV = Scalar(180.0, 255.0, 255.0) // (0-180, 0-255, 0-255)

    init {
        when (color) {
            Color.RED-> {
                this.lowHSV = redLowHSV
                this.highHSV = redHighHSV
                this.threshold = 0.15
            }
            Color.BLUE -> {
                this.lowHSV = blueLowHSV
                this.highHSV = blueHighHSV
                this.threshold = 0.15
            }
            else -> {
                this.lowHSV = Scalar(0.0, 180.0, 160.0)
                this.highHSV = Scalar(5.0, 230.0, 240.0)
                this.threshold = 0.15
            }
        }
    }
}