package org.firstinspires.ftc.teamcode.lib.math

class Vector(var x: Double, var y: Double, var z: Double, var unit: DistanceUnit) {

    constructor(x: Double, y: Double) : this(x, y, 0.0, DistanceUnit.CENTIMETER)
    constructor(x: Double, y: Double, z: Double) : this(x, y, z, DistanceUnit.CENTIMETER)

    fun clone(): Vector {
        return Vector(x, y, z, unit)
    }

    private fun convert(unit: DistanceUnit): Vector {
        this.x *= unit.multiplier / this.unit.multiplier
        this.y *= unit.multiplier / this.unit.multiplier
        this.z *= unit.multiplier / this.unit.multiplier
        this.unit = unit

        return this
    }

    fun toMM(): Vector {
        return this.convert(DistanceUnit.MILLIMETER)
    }

    fun toCM(): Vector {
        return this.convert(DistanceUnit.CENTIMETER)
    }

    fun toM(): Vector {
        return this.convert(DistanceUnit.METER)
    }

    fun add(vec: Vector): Vector {
        val v = vec.clone().convert(this.unit)
        this.x += v.x
        this.y += v.y
        this.z += v.z

        return this
    }

    fun sub(vec: Vector): Vector {
        val v = vec.clone().convert(this.unit)
        this.x -= v.x
        this.y -= v.y
        this.z -= v.z

        return this
    }

    fun multiply(vec: Vector): Vector {
        val v = vec.clone().convert(this.unit)
        this.x *= v.x
        this.y *= v.y
        this.z *= v.z

        return this
    }

    fun divide(vec: Vector): Vector {
        val v = vec.clone().convert(this.unit)
        this.x /= v.x
        this.y /= v.y
        this.z /= v.z

        return this
    }

    override fun toString(): String {
        return "[%.2f, %.2f, %.2f] ${this.unit}".format(this.x, this.y, this.z)
    }
}