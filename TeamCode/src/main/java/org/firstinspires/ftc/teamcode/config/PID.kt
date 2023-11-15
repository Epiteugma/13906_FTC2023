package org.firstinspires.ftc.teamcode.config

import org.firstinspires.ftc.teamcode.lib.PIDController

object PID {
    val motorCoefficients = mapOf(
            "frontLeft" to PIDController.Coefficients(),
            "backLeft" to PIDController.Coefficients(),
            "frontRight" to PIDController.Coefficients(),
            "backRight" to PIDController.Coefficients()
    )

    val angleCoefficients = PIDController.Coefficients()
}