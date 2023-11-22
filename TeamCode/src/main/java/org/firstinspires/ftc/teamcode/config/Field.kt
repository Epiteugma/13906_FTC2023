package org.firstinspires.ftc.teamcode.config

import org.firstinspires.ftc.teamcode.lib.Vision
import org.firstinspires.ftc.teamcode.lib.math.Vector

object Field {
    val tileSize = 24 * 2.54
    val size = this.tileSize * 6

    object Backdrop {
        val width = 25.625 * 2.54
        val depth = 11.25 * 2.54
        val offset = Field.tileSize * 1.5 - this.width / 2
        val aprilTagOffset = 14.0
        val aprilTagSpacing = 9.0
        val aprilTagSize = 6.0
    }

    object Camera {
        val lensIntrinsics = Vision.LensIntrinsics(817.819, 817.819, 321.062, 238.135)
        val decimation = 2f
    }

    val aprilTags = mapOf(
            1 to Vector(
                    -0.5 * this.size + Backdrop.offset + Backdrop.aprilTagOffset + Backdrop.aprilTagSize / 2,
                    this.size - Backdrop.offset
            ),
            2 to Vector(
                    -0.5 * this.size + Backdrop.offset + Backdrop.aprilTagOffset + Backdrop.aprilTagSize + Backdrop.aprilTagSpacing + Backdrop.aprilTagSize / 2,
                    this.size - Backdrop.offset
            ),
            3 to Vector(
                    -0.5 * this.size + Backdrop.offset + Backdrop.aprilTagOffset + (Backdrop.aprilTagSize + Backdrop.aprilTagSpacing) * 2 + Backdrop.aprilTagSize / 2,
                    this.size - Backdrop.offset
            ),
            4 to Vector(
                    0.5 * this.size - Backdrop.offset + Backdrop.aprilTagOffset + (Backdrop.aprilTagSize + Backdrop.aprilTagSpacing) * 2 + Backdrop.aprilTagSize / 2,
                    this.size - Backdrop.offset
            ),
            5 to Vector(
                    0.5 * this.size - Backdrop.offset + Backdrop.aprilTagOffset + Backdrop.aprilTagSize + Backdrop.aprilTagSpacing + Backdrop.aprilTagSize / 2,
                    this.size - Backdrop.offset
            ),
            6 to Vector(
                    0.5 * this.size - Backdrop.offset + Backdrop.aprilTagOffset + Backdrop.aprilTagSize / 2,
                    this.size - Backdrop.offset
            )
    )
}