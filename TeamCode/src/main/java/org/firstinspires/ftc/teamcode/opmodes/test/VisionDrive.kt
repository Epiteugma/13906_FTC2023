package org.firstinspires.ftc.teamcode.opmodes.test

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.config.Field
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.lib.PIDController
import org.firstinspires.ftc.teamcode.lib.TickLocalizer
import org.firstinspires.ftc.teamcode.lib.Vision
import org.firstinspires.ftc.teamcode.lib.math.MathUtil
import org.firstinspires.ftc.teamcode.lib.math.Vector
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

@Autonomous(name = "Vision Drive", group = "FTC23")
class VisionDrive : LinearOpMode() {
    private val vision = Vision()
    private val tickLocalizer = TickLocalizer.Builder()
            .setTicksPerRev(Robot.ticksPerRev * Robot.driveGearbox)
            .setWheelRadius(Robot.wheelRadius)
            .build()
    private val driveControllerLeft = PIDController()
    private val driveControllerRight = PIDController()
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
        vision.setCameraOffset(Robot.cameraOffset)
        Robot.initDashboardStream(vision.instance, 30.0)

        for(i in 0..<Robot.motors.size) {
            if(!Robot.drivetrainLeft.contains(i) && !Robot.drivetrainRight.contains(i)) continue
            tickLocalizer.addMotor(Robot.motors.values.elementAt(i))
        }

        waitForStart()

        var lastSample = System.currentTimeMillis()
        var wasLatest: Boolean
        while (opModeIsActive()) {
            if(System.currentTimeMillis() - lastSample < pidRate) continue
            lastSample += pidRate
            wasLatest = vision.latestData
            vision.updatePosition()
            val imuHeading = -Robot.imu.angularOrientation.firstAngle.toDouble()

            if(wasLatest && !vision.latestData) tickLocalizer.reset(vision.fieldRelativePosition.clone())
            if(!vision.latestData) tickLocalizer.update(imuHeading * Math.PI / 180)

            val lastUpdatedPosition = if(vision.latestData) vision.fieldRelativePosition else tickLocalizer.position

            var angleError = MathUtil.normalize(imuHeading - 90 + atan2(target.y - lastUpdatedPosition.y, target.x - lastUpdatedPosition.x) * 180 / Math.PI)
            if(angleError < -90 || angleError > 90) angleError = MathUtil.normalize(180 - angleError)
            angleError  *= Math.PI / 180 // to radians

            // Every rotation = 0.25 power
            driveControllerLeft.update(Robot.cmToTicks(lastUpdatedPosition.y - target.y) / (Robot.ticksPerRev * Robot.driveGearbox * 4), pidRate)
            driveControllerRight.update(Robot.cmToTicks(lastUpdatedPosition.y - target.y) / (Robot.ticksPerRev * Robot.driveGearbox * 4), pidRate)

            driveControllerLeft.o -= sin(angleError) / 4
            driveControllerRight.o += sin(angleError) / 4

            for(i in 0..<Robot.motors.size) {
                if(!Robot.drivetrainLeft.contains(i) && !Robot.drivetrainRight.contains(i)) continue
                val motor = Robot.motors.values.elementAt(i)

                motor.power = //if(vision.latestData) {
                    if(Robot.drivetrainLeft.contains(i)) driveControllerLeft.o
                    else driveControllerRight.o
//                }
//                else 0.0
            }

            telemetry.addData("Latest data", vision.latestData)
            telemetry.addData("Heading", imuHeading)
            telemetry.addData("Angle Error", angleError * 180 / Math.PI)
            telemetry.addData("Field pos", vision.fieldRelativePosition)
            telemetry.addData("TickLocalizer", tickLocalizer.position)
            telemetry.addData("DriveControllerLeft", "%.2f".format(driveControllerLeft.o))
            telemetry.addData("DriveControllerRight", "%.2f".format(driveControllerRight.o))
            telemetry.update()
        }
    }
}