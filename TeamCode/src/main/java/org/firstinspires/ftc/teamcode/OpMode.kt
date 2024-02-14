package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor

class MotorColl {
    lateinit var left: DcMotor
    lateinit var right: DcMotor
}

class Drivetrain {
    val front = MotorColl()
    val back = MotorColl()
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
    lateinit var motor:DcMotor
    var limits = arrayOfNulls<TouchSensor>(2)
}

class Claw {
    lateinit var left:Servo
    lateinit var right:Servo

    fun close() {
        this.left.position = 0.5
        this.right.position = 0.3
    }

    fun open() {
        this.left.position = 0.3
        this.right.position = 0.5
    }
}

abstract class OpMode : LinearOpMode() {
    val drivetrain = Drivetrain()
    val slides = Slides()
    var slideTilter = SlideTitler()
    var claw = Claw()

    lateinit var arm: DcMotor

    lateinit var clawPivot: Servo

    override fun runOpMode() {
        this.setup()
        waitForStart()
        while (opModeIsActive()) this.run()
        this.cleanup()
    }

    open fun setup() {
        this.drivetrain.front.left = this.hardwareMap.get(DcMotor::class.java, "frontLeft")
        this.drivetrain.front.right = this.hardwareMap.get(DcMotor::class.java, "frontRight")
        this.drivetrain.front.right.direction = DcMotorSimple.Direction.REVERSE
        this.drivetrain.back.left = this.hardwareMap.get(DcMotor::class.java, "backLeft")
        this.drivetrain.back.right = this.hardwareMap.get(DcMotor::class.java, "backRight")
        this.drivetrain.back.right.direction = DcMotorSimple.Direction.REVERSE

        this.slides.left.motor = this.hardwareMap.get(DcMotor::class.java, "leftSlide")
        this.slides.left.limits[0] = this.hardwareMap.get(TouchSensor::class.java,
            "magneticLimDownLeft")
        this.slides.left.limits[1] = this.hardwareMap.get(TouchSensor::class.java,
            "magneticLimUpLeft")

        this.slides.right.motor = this.hardwareMap.get(DcMotor::class.java, "rightSlide")
        this.slides.right.motor.direction = DcMotorSimple.Direction.REVERSE
        this.slides.right.limits[0] = this.hardwareMap.get(TouchSensor::class.java,
            "magneticLimDownRight")
        this.slides.right.limits[1] = this.hardwareMap.get(TouchSensor::class.java,
            "magneticLimUpRight")

        this.slideTilter.motor = this.hardwareMap.get(DcMotor::class.java, "slideTilter")
        this.slideTilter.limits[0] = this.hardwareMap.get(TouchSensor::class.java, "limBack")
        this.slideTilter.limits[1] = this.hardwareMap.get(TouchSensor::class.java, "limFront")


        this.arm = this.hardwareMap.get(DcMotor::class.java, "arm")

        this.clawPivot = this.hardwareMap.get(Servo::class.java, "clawPivot")

        this.claw.right = this.hardwareMap.get(Servo::class.java, "rightClaw")
        this.claw.left = this.hardwareMap.get(Servo::class.java, "leftClaw")

        this.claw.open()
    }
    abstract fun run()
    open fun cleanup() { }
}