package org.firstinspires.ftc.teamcode.lib.vision.detection

import org.firstinspires.ftc.teamcode.Color
import org.opencv.core.Scalar

class Prop(private val color: Color) {
    val lowHSV: Scalar
    val highHSV: Scalar
    val threshold: Double

    init {
        when (color) {
            Color.RED-> {
                this.lowHSV = Scalar(0.0, 180.0, 160.0)
                this.highHSV = Scalar(5.0, 230.0, 240.0)
                this.threshold = 0.15
            }
            Color.BLUE -> {
                this.lowHSV = Scalar(0.0, 180.0, 160.0)
                this.highHSV = Scalar(5.0, 230.0, 240.0)
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