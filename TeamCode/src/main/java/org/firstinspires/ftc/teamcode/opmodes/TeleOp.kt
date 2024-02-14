package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
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

        val slide = 0.3
        val slideHold = 1.0

        val arm = 0.45
        val armHold = 0.15

        var slideTilter = 0.4
    }

    class LastPositions {
        var arm = 0
    }

    private val mlt = Multipliers()
    private val lastPositions = LastPositions()

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
                drivePower + turnPower + strafePower
        this.drivetrain.front.right.power =
                drivePower - turnPower - strafePower
        this.drivetrain.back.left.power =
                drivePower + turnPower - strafePower
        this.drivetrain.back.right.power =
                drivePower - turnPower + strafePower

        this.telemetry.addLine("DRIVETRAIN")
        this.telemetry.addData("front left", "%.2f".format(this.drivetrain.front.left.power))
        this.telemetry.addData("front right", "%.2f".format(this.drivetrain.front.right.power))
        this.telemetry.addData("back left", "%.2f".format(this.drivetrain.back.left.power))
        this.telemetry.addData("back right", "%.2f".format(this.drivetrain.back.right.power))
        this.telemetry.addLine()

        this.telemetry.addLine("SLIDE TILTER")
        this.telemetry.addData("power", "%.2f".format(this.slideTilter.motor.power))
        this.telemetry.addData("limit back", this.slideTilter.limits[0]!!.isPressed)
        this.telemetry.addData("limit front", this.slideTilter.limits[1]!!.isPressed)
        this.telemetry.addLine()

        this.telemetry.addLine("SLIDES")
        this.telemetry.addLine("LEFT")
//        this.telemetry.addData("Position", this.slides.left.motor.currentPosition)
        this.telemetry.addData("power", "%.2f".format(this.slides.left.motor.power))
        this.telemetry.addData("limit down", this.slides.left.limits[0]!!.isPressed)
        this.telemetry.addData("limit up", this.slides.left.limits[1]!!.isPressed)
        this.telemetry.addLine("RIGHT")
//        this.telemetry.addData("Position", this.slides.right.motor.currentPosition)
        this.telemetry.addData("power", "%.2f".format(this.slides.right.motor.power))
        this.telemetry.addData("limit down", this.slides.right.limits[0]!!.isPressed)
        this.telemetry.addData("limit up", this.slides.right.limits[1]!!.isPressed)
        this.telemetry.addLine()
        
        this.telemetry.addLine("ARM")
        this.telemetry.addData("arm current pos", this.arm.currentPosition)
        this.telemetry.addData("arm target pos", this.arm.targetPosition)
        this.telemetry.addData("arm power", this.arm.power)
        this.telemetry.addLine()

        this.telemetry.addLine("CLAW")
        this.telemetry.addData("left pos", this.claw.left.position)
        this.telemetry.addData("right pos", this.claw.right.position)

        this.telemetry.update()
    }

    private fun setupT() {
        // RESET SLIDES
        this.slides.left.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        this.slides.right.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        // RESET ARM
        this.arm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        UTILS.resetEncoder(this.arm)

        // LOCK ARM
        UTILS.lockMotor(this.arm, this.mlt.armHold, this.lastPositions.arm)
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

        // MOVE SLIDES
        this.slides.left.motor.power = left
        this.slides.right.motor.power = right

        var slideTilterPower = -this.gamepad1.right_stick_y * this.mlt.slideTilter

        if (slideTilterPower > 0 && slideTilter.limits[1]!!.isPressed) slideTilterPower = 0.0
        if (slideTilterPower < 0 && slideTilter.limits[0]!!.isPressed) slideTilterPower = 0.0

        this.slideTilter.motor.power = slideTilterPower

        if(this.gamepad2.right_bumper)
            this.claw.close()
        else if(this.gamepad2.left_bumper)
            this.claw.open()

        var armPower = -this.gamepad2.right_stick_y
        if (abs(armPower) > this.mlt.armHold) {
            UTILS.unlockMotor(this.arm, armPower * this.mlt.arm)
            this.lastPositions.arm = this.arm.currentPosition
        } else UTILS.lockMotor(this.arm, this.mlt.armHold, this.lastPositions.arm)
    }
}