package org.firstinspires.ftc.teamcode.lib

import com.qualcomm.robotcore.hardware.DcMotor

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

    fun lockMotor(motor: DcMotor, holdPower: Double): Int {
        motor.targetPosition = motor.currentPosition
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.power = holdPower
        return motor.currentPosition
    }
}