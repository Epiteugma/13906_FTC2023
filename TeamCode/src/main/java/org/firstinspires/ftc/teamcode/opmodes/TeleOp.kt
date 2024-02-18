package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.OpMode
import org.firstinspires.ftc.teamcode.lib.UTILS
import kotlin.concurrent.thread
import kotlin.math.abs

@TeleOp(name = "FTC 2023", group = "FTC23")
class TeleOp : OpMode() {
    @Config
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

        val slideTilter = 0.5
        var slideTilterHold = 0.0

        val arm = 0.35
        val armHold = 0.15

        val clawPivotPower = 0.4
    }

    class LastPositions {
        var arm = 0
        var leftSlide = 0
        var rightSlide = 0
        var slideTilter = 0
    }

    private val mlt = Multipliers()
    private val lastPositions = LastPositions()

    private var globalToggleLock = false
    
    override fun setup() {
        super.setup()

        // DRIVER 2 THREAD;
        // DRIVER 2 RUNS ON MAIN
        thread {
            this.setupT()
            waitForStart()
            while (opModeIsActive())
                this.runT()
        }
    }

    override fun run() {
        this.moveRobot()
        this.printTelemetry()
    }

    private fun setupT() {
        this.resetSlides()
        this.resetArm()
    }

    private fun runT() {
        this.toggleGlobalPower()

        this.moveSlides()
        this.moveClaw()
        this.moveArm()

        // PLANE LAUNCHER
        if (this.gamepad1.touchpad)
            this.planeLauncher.position = 0.5

//        if(this.gamepad2.triangle){
//            if (buttonStates2.triangle){
//                mlt.slide = 0.5
//                mlt.slideHold = 0.3
//                buttonStates2.triangle = true
//            }
//            else {
//                mlt.slide = 0.8
//                mlt.slideHold = 0.2
//                buttonStates2.triangle = false
//            }
//        }
    }

    private fun resetSlides() {
        this.slides.left.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        this.slides.right.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        UTILS.resetEncoder(this.slides.left.motor)
        UTILS.resetEncoder(this.slides.right.motor)
        UTILS.lockMotor(
            this.slides.left.motor,
            this.mlt.slideHold,
            this.lastPositions.leftSlide
        )
        UTILS.lockMotor(
            this.slides.right.motor,
            this.mlt.slideHold,
            this.lastPositions.rightSlide
        )

        // SLIDE TILTER
        this.slideTilter.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        UTILS.resetEncoder(this.slideTilter.motor)
//        UTILS.lockMotor(
//            this.slideTilter.motor,
//            this.mlt.slideTilterHold,
//            this.lastPositions.slideTilter
//        )
    }

    private fun resetArm() {
        this.arm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        UTILS.resetEncoder(this.arm)
        UTILS.lockMotor(this.arm, this.mlt.armHold, this.lastPositions.arm)
    }

    private fun toggleGlobalPower() {
        if (gamepad1.cross && !this.globalToggleLock)
            this.mlt.global =
                if (this.mlt.global == 1.0) 0.5
                else 1.0
        this.globalToggleLock = gamepad1.cross
    }

    private fun moveRobot() {
        // CALCULATE DRIVE POWERS
        val drivePower = -this.gamepad1.left_stick_y * this.mlt.drive * this.mlt.global
        val turnPower = this.gamepad1.right_stick_x* this.mlt.turn * this.mlt.global
        val strafePower = this.gamepad1.left_stick_x * this.mlt.strafe * this.mlt.global

        // DRIVETRAIN
        this.drivetrain.front.left.power =
            (drivePower + turnPower + strafePower) * this.mlt.frontLeft
        this.drivetrain.front.right.power =
            (drivePower - turnPower - strafePower) * this.mlt.frontRight
        this.drivetrain.back.left.power =
            (drivePower + turnPower - strafePower) * this.mlt.backLeft
        this.drivetrain.back.right.power =
            (drivePower - turnPower + strafePower) * this.mlt.backRight
    }

    private fun moveSlides() {
        val slidePower = -this.gamepad2.left_stick_y * this.mlt.slide * this.mlt.global

        var left = slidePower
        var right = slidePower

        // LOWER LIMIT SWITCHES
        if (left < 0 && this.slides.left.limits[0]!!.isPressed) left = 0.0
        if (right < 0 && this.slides.right.limits[0]!!.isPressed) right = 0.0

        // UPPER LIMIT SWITCHES
        if (left > 0 && this.slides.left.limits[1]!!.isPressed) left = 0.0
        if (right > 0 && this.slides.right.limits[1]!!.isPressed) right = 0.0

        // MOVE SLIDES
        if (abs(left) > this.mlt.slideHold) {
            UTILS.unlockMotor(this.slides.left.motor, left)
            this.lastPositions.leftSlide = this.slides.left.motor.currentPosition
        }
        else
            UTILS.lockMotor(this.slides.left.motor, this.mlt.slideHold, this.lastPositions.leftSlide)

        if (abs(right) > this.mlt.slideHold) {
            UTILS.unlockMotor(this.slides.right.motor, right)
            this.lastPositions.rightSlide = this.slides.right.motor.currentPosition
        }
        else
            UTILS.lockMotor(this.slides.right.motor, this.mlt.slideHold, this.lastPositions.rightSlide)

        var slideTilterPower =
            (this.gamepad1.right_trigger - this.gamepad1.left_trigger) *
                    this.mlt.slideTilter *
                    this.mlt.global

        // TILTER LIMIT SWITCHES
        if (slideTilterPower > 0 && slideTilter.limits[1]!!.isPressed) slideTilterPower = 0.0
        if (slideTilterPower < 0 && slideTilter.limits[0]!!.isPressed) slideTilterPower = 0.0

        // MOVE SLIDE TILTER
        if (abs(slideTilterPower) > this.mlt.slideTilterHold)
            this.slideTilter.motor.power = slideTilterPower
        else this.slideTilter.motor.power = 0.0
    }

    private fun moveClaw() {
        if(this.gamepad2.right_bumper)
            this.claw.close()
        else if(this.gamepad2.left_bumper)
            this.claw.open()

        // CLAW PIVOT
        if (this.gamepad2.dpad_up) this.clawPivot.power = this.mlt.clawPivotPower
        else if (this.gamepad2.dpad_down) this.clawPivot.power = -this.mlt.clawPivotPower
        else this.clawPivot.power = 0.0
    }

    private fun moveArm() {
        val armPower = -this.gamepad2.right_stick_y * this.mlt.arm * this.mlt.global

        if (abs(armPower) > this.mlt.armHold) {
            UTILS.unlockMotor(this.arm, armPower)
            this.lastPositions.arm = this.arm.currentPosition
        } else UTILS.lockMotor(this.arm, this.mlt.armHold, this.lastPositions.arm)
    }

    private fun printTelemetry() {
        this.telemetry.addLine("DRIVETRAIN")
        this.telemetry.addData("Front Left", "%.2f".format(this.drivetrain.front.left.power))
        this.telemetry.addData("Front Right", "%.2f".format(this.drivetrain.front.right.power))
        this.telemetry.addData("Back Left", "%.2f".format(this.drivetrain.back.left.power))
        this.telemetry.addData("Back Right", "%.2f".format(this.drivetrain.back.right.power))
        this.telemetry.addLine()

        this.telemetry.addLine("SLIDE TILTER")
        this.telemetry.addData("Last Position", this.lastPositions.slideTilter)
        this.telemetry.addData("Current Position", this.slideTilter.motor.currentPosition)
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
        this.telemetry.addData("Power", this.arm.power)
        this.telemetry.addLine()

        this.telemetry.addLine("CLAW")
        this.telemetry.addData("Claw Pivot Power", this.clawPivot.power)
        this.telemetry.addData("Left Position", this.claw.left.position)
        this.telemetry.addData("Right Position", this.claw.right.position)

        this.telemetry.update()
    }
}