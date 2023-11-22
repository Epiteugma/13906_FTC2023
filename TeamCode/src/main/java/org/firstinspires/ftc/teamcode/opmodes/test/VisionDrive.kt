package org.firstinspires.ftc.teamcode.opmodes.test

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.config.Field
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.lib.PIDController
import org.firstinspires.ftc.teamcode.lib.Vision
import org.firstinspires.ftc.teamcode.lib.math.Vector

@Autonomous(name = "Vision Drive", group = "FTC23")
class VisionDrive : LinearOpMode() {
    private val vision = Vision()
    private val driveController = PIDController()
    private val pidRate = 100L

    private val target = Vector(0.0, 0.0)

    override fun runOpMode() {
        Robot.initHardware(hardwareMap)
        this.telemetry = Robot.bindDashboard(telemetry)
        vision.init(
                Field.Camera.lensIntrinsics,
                Field.Camera.decimation,
                Robot.camera,
                Field.aprilTags
        )
        waitForStart()

        var lastSample = System.currentTimeMillis()
        while (opModeIsActive()) {
            if(System.currentTimeMillis() - lastSample < pidRate) continue
            lastSample += pidRate

            // Every rotation = 0.25 power
            driveController.update(Robot.cmToTicks(target.x - vision.fieldRelativePosition.x).toDouble() / (Robot.ticksPerRev * Robot.driveGearbox * 4), pidRate)

            for(i in 0..<Robot.motors.size) {
                if(!Robot.drivetrainLeft.contains(i) && !Robot.drivetrainRight.contains(i)) continue
                val motor = Robot.motors.values.elementAt(i)

                motor.power = if(vision.latestData) driveController.o
                else 0.0
            }
        }
    }
}