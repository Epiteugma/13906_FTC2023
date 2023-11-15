package org.firstinspires.ftc.teamcode.lib

class PIDController(var kP: Double = 0.0, var kI: Double = 0.0, var kD: Double = 0.0) {
    var p = 0.0
    var i = 0.0
    var it = 0.0
    var d = 0.0
    var o = 0.0
    var pe = 0.0

    fun update(error: Double, delta: Long): Result {
        this.p = this.kP * error
        this.i = this.kI * error * delta
        this.it += this.i
        this.d = this.kD * (error - this.pe) / delta
        this.pe = error

        this.o = this.p + this.it + this.d
        return Result(this.p, this.it, this.d)
    }

    fun update(target: Double, current: Double, delta: Long): Result {
        return this.update(target - current, delta)
    }

    class Result(val p: Double, val it: Double, val d: Double) {
        val o = this.p + this.it + this.d
    }

    class Coefficients(val kP: Double = 1.0, val kI: Double = 0.0, val kD: Double = 0.0)
}