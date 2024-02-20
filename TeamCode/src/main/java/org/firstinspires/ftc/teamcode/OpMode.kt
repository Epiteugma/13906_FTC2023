package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.lib.Types

class Multipliers {
    var global = 1.0

    val drive = 0.7
    val turn = 0.65
    val strafe = 1.0

    val backRight = 1.0
    val frontRight = 1.0
    val backLeft = 1.2
    val frontLeft = 1.2

    var slideHold = 0.15
    var slide = 1.0

    val slideTilter = 1.0
    var slideTilterHold = 0.2

    val arm = 0.35
    val armHold = 0.15

    val clawPivotPower = 0.4
}

abstract class OpMode : LinearOpMode() {
    val drivetrain = Types.Drivetrain()
    val odometryEncoders = Types.OdometryEncoders()
    val slides = Types.Slides()
    var slideTilter = Types.SlideTitler()
    var claw = Types.Claw()
    val mlt = Multipliers()

    lateinit var planeLauncher: Servo

    lateinit var arm: DcMotor
    lateinit var clawPivot: CRServo

    private var stopRequested = false

    fun requestStop() {
        this.stopRequested = true
    }

    override fun runOpMode() {
        this.setup()
        waitForStart()
        while (opModeIsActive() && !stopRequested) this.run()
        this.cleanup()
    }

    open fun setup() {
        this.drivetrain.front.left = this.hardwareMap.get(DcMotor::class.java, "frontLeft")
        this.drivetrain.front.right = this.hardwareMap.get(DcMotor::class.java, "frontRight")
        this.drivetrain.front.right.direction = DcMotorSimple.Direction.REVERSE
        this.drivetrain.back.left = this.hardwareMap.get(DcMotor::class.java, "backLeft")
        this.drivetrain.back.right = this.hardwareMap.get(DcMotor::class.java, "backRight")
        this.drivetrain.back.right.direction = DcMotorSimple.Direction.REVERSE
        
        // odometry encoders
        this.odometryEncoders.left = this.drivetrain.front.left
        this.odometryEncoders.right = this.drivetrain.front.right
        this.odometryEncoders.center = this.drivetrain.back.left

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
        this.slideTilter.encoder = this.drivetrain.back.right
        this.slideTilter.limits[0] = this.hardwareMap.get(TouchSensor::class.java, "limBack")
        this.slideTilter.limits[1] = this.hardwareMap.get(TouchSensor::class.java, "limFront")


        this.arm = this.hardwareMap.get(DcMotor::class.java, "arm")

        this.clawPivot = this.hardwareMap.get(CRServo::class.java, "clawPivot")

        this.claw.right = this.hardwareMap.get(Servo::class.java, "rightClaw")
        this.claw.left = this.hardwareMap.get(Servo::class.java, "leftClaw")

        this.claw.close()

        this.planeLauncher = this.hardwareMap.get(Servo::class.java, "planeLauncher")
    }
    abstract fun run()
    open fun cleanup() { }
}