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

    fun moveToPosition(motor: DcMotor, targetPosition: Int, power: Double, errorThreshold: Int = 10) {

        while (abs(motor.currentPosition - targetPosition) > errorThreshold) {
            // wait for motor to reach position
            this.lockMotor(motor, power, targetPosition)
        }
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
        false, ticksPerRev: Int = 28, encoderMotor: DcMotor = motor) {
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        val dp = (savedPosition - encoderMotor.currentPosition) * if(invertEncoder) -1.0 else 1.0

        val calculatedHoldPower = holdPower * 1 / (1+exp(-abs(dp) / ticksPerRev))
        motor.power = if (dp > 0) calculatedHoldPower
            else if (dp < 0) -calculatedHoldPower
            else 0.0
    }
}