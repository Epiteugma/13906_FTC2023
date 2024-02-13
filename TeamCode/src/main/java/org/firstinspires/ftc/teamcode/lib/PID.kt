package org.firstinspires.ftc.teamcode.lib

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.max
import kotlin.math.min

class PID(kP: Double = 1.0, kI: Double = 0.0, kD: Double = 0.0) {
    private val timer = ElapsedTime()

    var p = 0.0
    var i = 0.0
    var it = 0.0
    var d = 0.0
    var o = 0.0
    var pe = 0.0

    var coeffs = Coefficients(kP, kI, kD)

    constructor(coefficients: Coefficients) : this() {
        this.coeffs = coefficients
    }

    fun update(error: Double): Result {
        this.p = this.coeffs.kP * error

        this.it += error * this.timer.seconds()
        this.it = min(max(-this.coeffs.integralLimit, this.it), this.coeffs.integralLimit)

        this.i = this.coeffs.kI * this.it
        this.d = this.coeffs.kD * ((error - this.pe) / this.timer.seconds())
        this.pe = error

        this.o = this.p + this.i + this.d
        this.timer.reset()
        return Result(this.p, this.i, this.d)
    }

    fun update(target: Double, current: Double): Result {
        return this.update(target - current)
    }

    class Result(val p: Double, val i: Double, val d: Double) {
        val o = this.p + this.i + this.d
    }

    class Coefficients(@JvmField var kP: Double = 1.0, @JvmField var kI: Double = 0.0, @JvmField var kD: Double = 0.0, @JvmField var integralLimit: Double = Double.POSITIVE_INFINITY)
}