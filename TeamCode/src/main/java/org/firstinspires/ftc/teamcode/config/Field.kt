package org.firstinspires.ftc.teamcode.config

import org.firstinspires.ftc.teamcode.lib.Vision
import org.firstinspires.ftc.teamcode.lib.math.Vector

object Field {
    val fieldTags = mapOf(
            1 to Vector(0.0, 0.0),
            2 to Vector(0.0, 0.0),
            3 to Vector(0.0, 0.0),
            4 to Vector(0.0, 0.0),
            5 to Vector(0.0, 0.0),
            6 to Vector(0.0, 0.0)
    )

    val lensIntrinsics = Vision.LensIntrinsics()//817.819, 817.819, 321.062, 238.135)
    val decimation = 2f
}