package org.firstinspires.ftc.teamcode.lib

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor

class Types {
    class MotorColl {
        lateinit var left: DcMotor
        lateinit var right: DcMotor
    }

    class Drivetrain {
        val front = MotorColl()
        val back = MotorColl()

        fun halt() {
            this.front.left.power = 0.0
            this.front.right.power = 0.0
            this.back.left.power = 0.0
            this.back.right.power = 0.0
        }
    }

    class OdometryEncoders {
        lateinit var left: DcMotor
        lateinit var right: DcMotor
        lateinit var center: DcMotor

        val ticksPerRev = 8192
        val wheelDiameter = 6.0 / 2.0
        val wheelCircumference = this.wheelDiameter * Math.PI
    }

    class Slides {
        val left = Slide()
        val right = Slide()
    }

    class Slide {
        lateinit var motor: DcMotor
        val limits = arrayOfNulls<TouchSensor>(2)
    }

    class SlideTitler {
        lateinit var motor: DcMotor
        lateinit var encoder: DcMotor
        var limits = arrayOfNulls<TouchSensor>(2)

        val ticksPerRev = 28
        val gearRatio = 5.0/3.0
        val slideTilterTicksPerRev = this.ticksPerRev * this.gearRatio
    }

    class Claw {
        lateinit var left: Servo
        lateinit var right: Servo

        fun close() {
            this.left.position = 0.45
            this.right.position = 0.25
        }

        fun open() {
            this.left.position = 0.1
            this.right.position = 0.7
        }
    }
}