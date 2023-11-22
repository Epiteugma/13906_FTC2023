package org.firstinspires.ftc.teamcode.opmodes.test

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorSimple.*
import org.firstinspires.ftc.teamcode.config.PID
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.lib.PIDController

@Autonomous(name = "PID Test", group = "FTC23")
class PID : LinearOpMode() {
    private val motorControllers = mutableMapOf<String, PIDController>()
    private var driveTarget = 0.0

    override fun runOpMode() {
        this.telemetry = Robot.bindDashboard(this.telemetry)
        Robot.initHardware(this.hardwareMap)
        for(motor in PID.motorCoefficients) motorControllers[motor.key] = PIDController(motor.value)

        waitForStart()

        var lastPIDUpdate = System.currentTimeMillis()
        var uLock = false
        var dLock = false
//        var lLock = false
//        var rLock = false

        while (opModeIsActive()) {
            val delta = System.currentTimeMillis() - lastPIDUpdate
            if(delta >= PID.updateRate) {
                this.updatePID(delta)
                lastPIDUpdate += delta
            }

            telemetry.addData("Drive Target", "%.2f".format(this.driveTarget))

            for(controller in this.motorControllers) {
                telemetry.addData("${controller.key} PID", "%.2f".format(controller.value.o))
                Robot.motors[controller.key]?.power = controller.value.o
            }

            if(gamepad1.dpad_up && !uLock) this.driveTarget += 10.0
            if(gamepad1.dpad_down && !dLock) this.driveTarget -= 10.0

            uLock = gamepad1.dpad_up
            dLock = gamepad1.dpad_down

            this.driveTarget += -gamepad1.left_stick_y / 50
            telemetry.update()
        }
    }

    private fun updatePID(delta: Long) {
        for(controller in this.motorControllers) {
            val motor = Robot.motors[controller.key]!!

            controller.value.update((Robot.cmToTicks(this.driveTarget) - motor.currentPosition) / (Robot.ticksPerRev * Robot.driveGearbox), delta)
        }
    }
}