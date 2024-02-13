package org.firstinspires.ftc.teamcode.lib

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.lib.math.*

class TickLocalizer(private val ticksPerRev: Int, private val wheelRadius: Double) {
    private val cmPerRev = 2 * Math.PI * wheelRadius
    var position = Vector(0.0, 0.0)
    private val motors = mutableListOf<DcMotor>()
    /*private*/ val motorPositions = mutableMapOf<DcMotor, Vector>()
    private val lastTicks = mutableMapOf<DcMotor, Int>()

    fun reset(vector: Vector = Vector(0.0, 0.0)) {
        this.position = vector.clone()
        motorPositions.clear()
    }

    fun addMotor(motor: DcMotor) {
        motors.add(motor)
    }

    fun update(heading: Double): Vector {

        var sumX = 0.0
        var sumY = 0.0

        for(motor in motors) {
            if(lastTicks[motor] == null) lastTicks[motor] = motor.currentPosition

            val delta = (motor.currentPosition - lastTicks[motor]!!)
            lastTicks[motor] = motor.currentPosition

            motorPositions[motor] = motorPositions[motor] ?: this.position.clone()
            motorPositions[motor]!!.x += delta * sin(heading)
            motorPositions[motor]!!.y += delta * cos(heading)

            sumX += motorPositions[motor]!!.x
            sumY += motorPositions[motor]!!.y
        }

        this.position.x = ticks2Cm(sumX.toInt(), wheelRadius, ticksPerRev) / motors.size
        this.position.y = ticks2Cm(sumY.toInt(), wheelRadius, ticksPerRev) / motors.size

        return this.position
    }
}