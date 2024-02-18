package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.OpMode
import org.firstinspires.ftc.teamcode.lib.Vision
import org.firstinspires.ftc.teamcode.lib.math.cap
import org.firstinspires.ftc.teamcode.lib.math.normalize
import kotlin.math.abs

@Autonomous(name = "FTC23 Romania", group = "FTC23")
class BasicAuton : OpMode() {
    private lateinit var imu: IMU
    private lateinit var vision: Vision
    private val timer = ElapsedTime()
    private val recognizeTimeout = 5_000
    private val turnErrorThreshold = 5

    override fun setup() {
        super.setup()

        this.imu = this.hardwareMap.get(IMU::class.java, "imu")

        val imuParams = IMU.Parameters(RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ))

        this.imu.initialize(imuParams)

//        this.vision = Vision()
//        this.vision.init(
//                Vision.LensIntrinsics(),
//                2f,
//                this.hardwareMap.get(WebcamName::class.java, "Webcam 1"),
//                mapOf()
//        )

        this.waitForStart()
        timer.reset()
    }

    private fun recognize(): PropPosition {
        return PropPosition.LEFT
    }

    private enum class PropPosition {
        NONE,
        CENTER,
        LEFT,
        RIGHT,
    }

    private fun navigate(heading: Double) {
        val start = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES
        ).thirdAngle

        var error = heading - start

        while(this.opModeIsActive() && abs(error) > turnErrorThreshold) {
            error = normalize(heading - imu.getRobotOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES
            ).thirdAngle)


            this.telemetry.addLine("Turning...")
            this.telemetry.addData("Target", heading)
            this.telemetry.addData("Current", normalize(heading - error))
            this.telemetry.addData("Error", error)
            this.telemetry.update()

            this.drivetrain.front.left.power = cap(-error / 90.0, -1.0, 1.0)
            this.drivetrain.front.right.power = cap(error / 90.0, -1.0, 1.0)
            this.drivetrain.back.left.power = cap(-error / 90.0, -1.0, 1.0)
            this.drivetrain.back.right.power = cap(error / 90.0, -1.0, 1.0)
        }
    }

    private fun placePixel() {
        // TODO: place pixel
    }

    override fun run() {
        var position = PropPosition.NONE

        while (this.opModeIsActive() &&
                position == PropPosition.NONE &&
                timer.milliseconds() < this.recognizeTimeout
        ) {
            telemetry.addLine("Recognition in progress...")
            telemetry.addData(
                    "Time remaining",
                    "%.2f".format((this.recognizeTimeout - timer.milliseconds()) / 1_000.0)
            )
            telemetry.update()

            position = this.recognize()
        }

        val targetAngle: Double

        when (position) {
            PropPosition.NONE, PropPosition.CENTER -> {
                targetAngle = 0.0
            }
            PropPosition.LEFT -> {
                targetAngle = -30.0
            }
            PropPosition.RIGHT -> {
                targetAngle = 30.0
            }
        }

        this.navigate(targetAngle)
        this.placePixel()
        this.requestOpModeStop()
    }
}