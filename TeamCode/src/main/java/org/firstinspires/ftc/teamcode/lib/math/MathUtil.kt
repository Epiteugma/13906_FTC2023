package org.firstinspires.ftc.teamcode.lib.math

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.floor

object MathUtil {
    fun normalize(angle: Double, unit: AngleUnit): Double {
        val halfRot = when(unit) {
            AngleUnit.DEGREES -> 180.0
            AngleUnit.RADIANS -> Math.PI
        }

        return this.modulo(angle + halfRot, 2 * halfRot) - halfRot
    }

    fun modulo(a: Double, n: Double): Double {
        return a - floor(a / n) * n
    }

    fun normalize(angle: Double): Double {
        return this.normalize(angle, AngleUnit.DEGREES)
    }
}