package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.OpMode
import org.firstinspires.ftc.teamcode.lib.UTILS
import kotlin.concurrent.thread
import kotlin.math.abs

@TeleOp(name = "FTC 2023", group = "FTC23")
class TeleOp : OpMode() {

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

        // SLIDE TILTER
        this.slideTilter.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        UTILS.resetEncoder(this.drivetrain.back.right)
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
        if (abs(slideTilterPower) > this.mlt.slideTilterHold) {
            UTILS.unlockMotor(this.slideTilter.motor, slideTilterPower)
            this.lastPositions.slideTilter = this.drivetrain.back.right.currentPosition
//            this.slideTilter.motor.power = slideTilterPower
        }
        else
//            this.slideTilter.motor.power = 0.0
            UTILS.lockMotor(
                    this.slideTilter.motor,
                    this.mlt.slideTilterHold,
                    this.lastPositions.slideTilter,
                    true,
                    8192,
                    this.drivetrain.back.right
            )
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
        } else UTILS.lockMotor(this.arm, this.mlt.armHold, this.lastPositions.arm, false)
    }
}