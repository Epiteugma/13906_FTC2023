package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.OpMode
import org.firstinspires.ftc.teamcode.lib.UTILS
import kotlin.concurrent.thread
import kotlin.math.abs

@TeleOp(name = "FTC 2023", group = "FTC23")
class TeleOp : OpMode() {
    class Multipliers {
        val drive = 1.0
        val turn = 1.0
        val strafe = 1.0

        val slide = 1.0
        val slideHold = 1.0

        val arm = 1.0
        val armHold = 1.0
    }

    private val mlt = Multipliers()

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
        // CALCULATE DRIVE POWERS
        val drivePower = -this.gamepad1.left_stick_y * this.mlt.drive
        val turnPower = this.gamepad1.right_stick_x* this.mlt.turn
        val strafePower = this.gamepad1.left_stick_x * this.mlt.strafe

        // DRIVETRAIN
        this.drivetrain.front.left.power =
                drivePower + turnPower - strafePower
        this.drivetrain.front.right.power =
                drivePower - turnPower + strafePower
        this.drivetrain.back.left.power =
                drivePower + turnPower + strafePower
        this.drivetrain.back.right.power =
                drivePower - turnPower - strafePower

        this.telemetry.addLine("DRIVETRAIN")
        this.telemetry.addData("front left", "%.2f".format(this.drivetrain.front.left.power))
        this.telemetry.addData("front right", "%.2f".format(this.drivetrain.front.right.power))
        this.telemetry.addData("back left", "%.2f".format(this.drivetrain.back.left.power))
        this.telemetry.addData("back right", "%.2f".format(this.drivetrain.back.right.power))

        this.telemetry.addLine()
        this.telemetry.addLine("SLIDES")
        this.telemetry.addData("left pos", this.slides.left.motor.currentPosition)
        this.telemetry.addData("left power", this.slides.left.motor.power)
        this.telemetry.addData("right pos", this.slides.right.motor.currentPosition)
        this.telemetry.addData("right power", this.slides.right.motor.power)

        this.telemetry.addLine()
        this.telemetry.addLine("ARM")
        this.telemetry.addData("arm pos", this.arm.currentPosition)
        this.telemetry.addData("arm power", this.arm.power)

        this.telemetry.update()
    }

    private fun setupT() {
        // RESET SLIDES, ARM
        UTILS.resetEncoder(this.slides.left.motor)
        UTILS.resetEncoder(this.slides.right.motor)
        UTILS.resetEncoder(this.arm)

        // LOCK SLIDES, ARM
        UTILS.lockMotor(this.slides.left.motor, this.mlt.slideHold * this.mlt.slide)
        UTILS.lockMotor(this.slides.right.motor, this.mlt.slideHold * this.mlt.slide)
        UTILS.lockMotor(this.arm, this.mlt.armHold * this.mlt.arm)
    }

    private fun runT() {
        val slidePower = -this.gamepad2.left_stick_y * this.mlt.slide

        var left = slidePower
        var right = slidePower

        // LOWER LIMIT SWITCHES
        if (left < 0 && this.slides.left.limits[0]!!.isPressed) left = 0.0
        if (right < 0 && this.slides.right.limits[0]!!.isPressed) right = 0.0

        // UPPER LIMIT SWITCHES
        if (left > 0 && this.slides.left.limits[1]!!.isPressed) left = 0.0
        if (right > 0 && this.slides.right.limits[1]!!.isPressed) right = 0.0

        // UNLOCK SLIDES
        if (abs(left) >= this.mlt.slide * this.mlt.slideHold)
            UTILS.unlockMotor(this.slides.left.motor, left)
        else
            UTILS.lockMotor(this.slides.left.motor, this.mlt.slideHold)

        if (abs(right) >= this.mlt.slide * this.mlt.slideHold)
            UTILS.unlockMotor(this.slides.right.motor, right)
        else
            UTILS.lockMotor(this.slides.right.motor, this.mlt.slideHold)
    }
}