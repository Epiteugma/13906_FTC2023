package org.firstinspires.ftc.teamcode.lib.math

import kotlin.math.floor

fun normalize(angle: Double, unit: AngleUnit): Double {
    val halfRot = unit.rotation / 2.0
    return modulo(angle + halfRot, 2 * halfRot) - halfRot
}

fun normalize(angle: Double): Double {
    return normalize(angle, AngleUnit.DEGREES)
}

fun modulo(a: Double, n: Double): Double {
    return a - floor(a / n) * n
}

fun sin(angle: Double, unit: AngleUnit): Double {
    return kotlin.math.sin(angle / unit.rotation * 2 * Math.PI)
}

fun sin(angle: Double): Double {
    return sin(angle, AngleUnit.DEGREES)
}

fun cos(angle: Double, unit: AngleUnit): Double {
    return kotlin.math.cos(angle / unit.rotation * 2 * Math.PI)
}

fun cos(angle: Double): Double {
    return cos(angle, AngleUnit.DEGREES)
}

fun tan(angle: Double, unit: AngleUnit): Double {
    return kotlin.math.tan(angle / unit.rotation * 2 * Math.PI)
}

fun tan(angle: Double): Double {
    return tan(angle, AngleUnit.DEGREES)
}

fun cm2Ticks(cm: Double, wheelRadius: Double, ticksPerRev: Int): Int {
    val revolutions = cm / (2 * Math.PI * wheelRadius);
    return (revolutions * ticksPerRev).toInt()
}

fun ticks2Cm(ticks: Int, wheelRadius: Double, ticksPerRev: Int): Double {
    val revolutions = ticks / ticksPerRev
    return revolutions * 2 * Math.PI * wheelRadius
}

fun cap(value: Double, min: Double, max: Double): Double {
    if(value < min) return min;
    if(value > max) return max;
    return value;
}

fun Boolean.asInt(): Int {
    return if(this) 1 else 0
}