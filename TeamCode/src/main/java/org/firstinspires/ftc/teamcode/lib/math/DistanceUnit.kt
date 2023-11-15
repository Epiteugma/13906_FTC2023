package org.firstinspires.ftc.teamcode.lib.math

enum class DistanceUnit {

    MILLIMETER(1.0),
    CENTIMETER(10.0),
    METER(1_000.0);

    val multiplier: Double
    constructor(multiplier: Double) {
        this.multiplier = multiplier
    }
}