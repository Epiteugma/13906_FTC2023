package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.lib.Types

class LastPositions {
    var arm = 0
    var leftSlide = 0
    var rightSlide = 0
    var slideTilter = 0
}

class DriveTrainMlts{
    var frontLeft = 1.0
    var backLeft = 1.0
    var frontRight = 0.9
    var backRight = 0.9
}

class Multipliers {

    var driveTrainMlts = DriveTrainMlts()

    var driver1Factor = 1.0

    val drive = 0.55
    val turn = 0.65
    val strafe = 0.65

    var slideHold = 0.15
    var slide = 1.0

    val slideTilter = 1.0
    var slideTilterHold = 0.2

    val arm = 0.35
    val armHold = 0.25

    val clawPivotPower = 0.4
}

abstract class OpMode : LinearOpMode() {
    val drivetrain = Types.Drivetrain()
    val odometryEncoders = Types.OdometryEncoders()
    val slides = Types.Slides()
    var slideTilter = Types.SlideTitler()
    var claw = Types.Claw()
    val mlt = Multipliers()
    val lastPositions = LastPositions()

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
        this.drivetrain.back.left.direction = DcMotorSimple.Direction.REVERSE
        this.drivetrain.back.right = this.hardwareMap.get(DcMotor::class.java, "backRight")
        this.drivetrain.back.right.direction = DcMotorSimple.Direction.REVERSE

        this.slides.left.motor = this.hardwareMap.get(DcMotor::class.java, "leftSlide")
        this.slides.left.limits[0] = this.hardwareMap.get(TouchSensor::class.java,
            "limDownLeft")
        this.slides.left.limits[1] = this.hardwareMap.get(TouchSensor::class.java,
            "limUpLeft")

        this.slides.right.motor = this.hardwareMap.get(DcMotor::class.java, "rightSlide")
        this.slides.right.motor.direction = DcMotorSimple.Direction.REVERSE
        this.slides.right.limits[0] = this.hardwareMap.get(TouchSensor::class.java,
            "limDownRight")
        this.slides.right.limits[1] = this.hardwareMap.get(TouchSensor::class.java,
            "limUpRight")

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

        // odometry encoders
        this.odometryEncoders.left = this.drivetrain.front.left
        this.odometryEncoders.right = this.drivetrain.front.right
        this.odometryEncoders.center = this.slideTilter.motor
    }

    fun printTelemetry() {
        this.telemetry.addLine("DRIVETRAIN")
        this.telemetry.addData(
            "Front Left",
            "%.2f | %d".format(this.drivetrain.front.left.power, this.drivetrain.front.left.currentPosition)
        )
        this.telemetry.addData(
            "Front Right",
            "%.2f | %d".format(this.drivetrain.front.right.power, this.drivetrain.front.right.currentPosition)
        )
        this.telemetry.addData(
            "Back Left",
            "%.2f | %d".format(this.drivetrain.back.left.power, this.drivetrain.back.left.currentPosition)
        )
        this.telemetry.addData(
            "Back Right",
            "%.2f | %d".format(this.drivetrain.back.right.power, this.drivetrain.back.right.currentPosition)
        )
        this.telemetry.addLine()

        this.telemetry.addLine("SLIDE TILTER")
        this.telemetry.addData("Last Position", this.lastPositions.slideTilter)
        this.telemetry.addData("Current Position", this.drivetrain.back.right.currentPosition)
        this.telemetry.addData("Power", "%.2f".format(this.slideTilter.motor.power))
        this.telemetry.addData("Limit Back", this.slideTilter.limits[0]!!.isPressed)
        this.telemetry.addData("Limit Front", this.slideTilter.limits[1]!!.isPressed)
        this.telemetry.addLine()

        this.telemetry.addLine("SLIDES")

        this.telemetry.addLine("LEFT")
        this.telemetry.addData("Last Position", this.lastPositions.leftSlide)
        this.telemetry.addData("Current Position", this.slides.left.motor.currentPosition)
        this.telemetry.addData("Power", "%.2f".format(this.slides.left.motor.power))
        this.telemetry.addData("Limit Down", this.slides.left.limits[0]!!.isPressed)
        this.telemetry.addData("Limit Up", this.slides.left.limits[1]!!.isPressed)

        this.telemetry.addLine("RIGHT")
        this.telemetry.addData("Last Position", this.lastPositions.rightSlide)
        this.telemetry.addData("Current Position", this.slides.right.motor.currentPosition)
        this.telemetry.addData("Power", "%.2f".format(this.slides.right.motor.power))
        this.telemetry.addData("Limit Down", this.slides.right.limits[0]!!.isPressed)
        this.telemetry.addData("Limit Up", this.slides.right.limits[1]!!.isPressed)
        this.telemetry.addLine()

        this.telemetry.addLine("ARM")
        this.telemetry.addData("Last Position", this.lastPositions.arm)
        this.telemetry.addData("Current Position", this.arm.currentPosition)
        this.telemetry.addData("Power", "%.2f".format(this.arm.power))
        this.telemetry.addLine()

        this.telemetry.addLine("CLAW")
        this.telemetry.addData("Claw Pivot Power", this.clawPivot.power)
        this.telemetry.addData("Left Position", this.claw.left.position)
        this.telemetry.addData("Right Position", this.claw.right.position)

        this.telemetry.update()
    }

    abstract fun run()
    open fun cleanup() { }
}