package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
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

abstract class OpMode : LinearOpMode() {
    val drivetrain = Drivetrain()
    val slides = Slides()

    lateinit var arm: DcMotor

    override fun runOpMode() {
        this.setup()
        waitForStart()
        while (opModeIsActive()) this.run()
        this.cleanup()
    }

    open fun setup() {
        this.drivetrain.front.left = this.hardwareMap.get(DcMotor::class.java, "frontLeft")
        this.drivetrain.front.right = this.hardwareMap.get(DcMotor::class.java, "frontRight")
        this.drivetrain.back.left = this.hardwareMap.get(DcMotor::class.java, "backLeft")
        this.drivetrain.back.right = this.hardwareMap.get(DcMotor::class.java, "backRight")

        this.slides.left.motor = this.hardwareMap.get(DcMotor::class.java, "leftSlide")
        this.slides.left.limits[0] = this.hardwareMap.get(TouchSensor::class.java, "lSlideDown")
        this.slides.left.limits[1] = this.hardwareMap.get(TouchSensor::class.java, "lSlideUp")

        this.slides.right.motor = this.hardwareMap.get(DcMotor::class.java, "rightSlide")
        this.slides.right.limits[0] = this.hardwareMap.get(TouchSensor::class.java, "rSlideDown")
        this.slides.right.limits[1] = this.hardwareMap.get(TouchSensor::class.java, "rSlideUp")

        this.arm = this.hardwareMap.get(DcMotor::class.java, "arm")
    }
    abstract fun run()
    open fun cleanup() { }
}