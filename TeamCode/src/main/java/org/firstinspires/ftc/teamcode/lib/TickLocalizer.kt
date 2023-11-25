package org.firstinspires.ftc.teamcode.lib

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.lib.math.Vector
import kotlin.math.cos
import kotlin.math.sin

class TickLocalizer(private val ticksPerRev: Double, private val wheelRadius: Double) {
    private val cmPerRev = 2 * Math.PI * wheelRadius
    var position = Vector(0.0, 0.0)
    private val motors = mutableListOf<DcMotor>()
    private val motorPositions = mutableMapOf<DcMotor, Vector>()
    private val lastTicks = mutableMapOf<DcMotor, Int>()

    class Builder {
        private var ticksPerRev = 28.0
        private var wheelRadius = 1.0

        fun setTicksPerRev(ticksPerRev: Double): Builder {
            this.ticksPerRev = ticksPerRev
            return this
        }

        fun setWheelRadius(wheelRadius: Double): Builder {
            this.wheelRadius = wheelRadius
            return this
        }

        fun build(): TickLocalizer {
            return TickLocalizer(ticksPerRev, wheelRadius)
        }
    }

    fun reset(vector: Vector = Vector(0.0, 0.0)) {
        this.position = vector.clone()
        motorPositions.clear()
    }

    fun addMotor(motor: DcMotor) {
        motors.add(motor)
    }

    private fun ticks2Cm(ticks: Int): Double {
        return ticks / ticksPerRev * cmPerRev
    }

    fun update(headingRadians: Double) {
        var sumX = 0.0
        var sumY = 0.0

        for(motor in motors) {
            if(lastTicks[motor] == null) lastTicks[motor] = motor.currentPosition

            val delta = (motor.currentPosition - lastTicks[motor]!!)
            lastTicks[motor] = motor.currentPosition

            motorPositions[motor] = motorPositions[motor] ?: this.position.clone()
            motorPositions[motor]!!.x += ticks2Cm(delta) * sin(headingRadians)
            motorPositions[motor]!!.y += ticks2Cm(delta) * cos(headingRadians)

            sumX += motorPositions[motor]!!.x
            sumY += motorPositions[motor]!!.y
        }

        this.position.x = sumX / motors.size
        this.position.y = sumY / motors.size
    }
}