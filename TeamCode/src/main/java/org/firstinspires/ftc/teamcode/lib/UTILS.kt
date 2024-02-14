package org.firstinspires.ftc.teamcode.lib

import com.qualcomm.robotcore.hardware.DcMotor
import kotlin.math.exp
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

object UTILS {
    private fun resetEncoder(motor: DcMotor, newMode: DcMotor.RunMode) {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = newMode
    }

    fun resetEncoder(motor: DcMotor) {
        this.resetEncoder(motor, motor.mode)
    }

    fun unlockMotor(motor: DcMotor, power: Double) {
        motor.power = power
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

//    fun lockMotor(motor: DcMotor, holdPower: Double): Int {
//        if (motor.mode == DcMotor.RunMode.RUN_TO_POSITION)
//            return motor.currentPosition
//
//        motor.targetPosition = motor.currentPosition
//        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
//        motor.power = holdPower
//        return motor.currentPosition
//    }

    fun lockMotor(motor: DcMotor, holdPower: Double, savedPosition: Int, invertEncoder: Boolean =
        false) {
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        val dp = savedPosition - (motor.currentPosition * if(invertEncoder) -1.0 else 1.0)

        motor.power = if (dp > 0) holdPower
            else if (dp < 0) -holdPower
            else 0.0
    }
}