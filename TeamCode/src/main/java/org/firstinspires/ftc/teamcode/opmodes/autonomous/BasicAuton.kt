package org.firstinspires.ftc.teamcode.opmodes.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.OpMode
import org.firstinspires.ftc.teamcode.lib.UTILS
import org.firstinspires.ftc.teamcode.lib.math.cap
import org.firstinspires.ftc.teamcode.lib.math.normalize
import org.firstinspires.ftc.teamcode.lib.vision.detection.ItemDetector
import org.opencv.core.Scalar
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam
import kotlin.math.abs

@Autonomous(name = "FTC23 Romania", group = "FTC23")
class BasicAuton : OpMode() {
    private lateinit var imu: IMU
    private lateinit var detector: ItemDetector
    private val timer = ElapsedTime()
    private val recognizeTimeout = 5_000
    private val turnErrorThreshold:Double  = 5.0
    private val driveErrorThreshold:Double = 2.0
    private val width: Int = 640
    private val height: Int = 480

    class RedProp {
        val lowHSV = Scalar(0.0, 180.0, 160.0)
        val highHSV = Scalar(5.0, 230.0, 240.0)
        val threshold = 0.15
    }

    class BlueProp {
        val lowHSV = Scalar(0.0, 180.0, 160.0)
        val highHSV = Scalar(5.0, 230.0, 240.0)
        val threshold = 0.15
    }

    override fun setup() {
        super.setup()

        this.imu = this.hardwareMap.get(IMU::class.java, "imu")

        val imuParams = IMU.Parameters(RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ))

        this.imu.initialize(imuParams)
        this.imu.resetYaw()

        val prop = RedProp()

        val webcamName: WebcamName = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        val monitorViewIdParent = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val webcam: OpenCvWebcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, monitorViewIdParent)
        detector.init(width, height,
            prop.lowHSV,
            prop.highHSV,
            prop.threshold, telemetry)
        webcam.setPipeline(detector)
        webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT)
                FtcDashboard.getInstance().startCameraStream(webcam, 30.0)
            }

            override fun onError(errorCode: Int) {}
        })

        this.waitForStart()
        timer.reset()

        UTILS.resetEncoder(this.arm)
        UTILS.resetEncoder(this.slideTilter.encoder)
    }

    private fun turn(heading: Double, holdTime: Double = 2.0) {
        val start = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES
        ).thirdAngle

        var error = heading - start
        val holdTimer = ElapsedTime()
        var reachedTarget = false

        while(
            this.checkState() &&
            (!reachedTarget || holdTimer.milliseconds() < holdTime * 1000)
        ) {
            reachedTarget = reachedTarget || abs(error) < turnErrorThreshold
            if (!reachedTarget) holdTimer.reset()

            error = normalize(heading - imu.getRobotOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES
            ).thirdAngle)

            val base = if(reachedTarget) 0.0 else 0.5
            val mlt = 0.4

            val div = 360.0 * mlt
            val add =
                if (error < 0) -base
                else base

            this.drivetrain.front.left.power = cap(-error / div - add, -1.0, 1.0)
            this.drivetrain.front.right.power = cap(error / div + add, -1.0, 1.0)
            this.drivetrain.back.left.power = cap(-error / div - add, -1.0, 1.0)
            this.drivetrain.back.right.power = cap(error / div + add, -1.0, 1.0)

            this.telemetry.addLine("Turning...")
            this.telemetry.addData("Target", heading)
            this.telemetry.addData("Current", normalize(heading - error))
            this.telemetry.addData("Error", error)
            telemetry.addData("Power", error / div + add);
            this.telemetry.update()
        }
        
        this.drivetrain.halt()
    }

    private fun driveForward(distance: Double, holdTime: Double = 3.0) {
        UTILS.resetEncoder(this.odometryEncoders.left)
        UTILS.resetEncoder(this.odometryEncoders.right)

        val ticksToRun = distance / this.odometryEncoders.wheelCircumference * this
            .odometryEncoders.ticksPerRev
        val errorThreshold = this.driveErrorThreshold / this.odometryEncoders.wheelCircumference * this
            .odometryEncoders.ticksPerRev
        var reachedTarget = false
        val holdTimer = ElapsedTime()

        while (
            this.checkState() &&
            (!reachedTarget || holdTimer.milliseconds() < holdTime * 1000)
        ) {
            reachedTarget = reachedTarget || (abs(ticksToRun - this.odometryEncoders.left
            .currentPosition) <
                    errorThreshold &&
                    abs(ticksToRun - this.odometryEncoders.right.currentPosition) < errorThreshold)
            if (!reachedTarget) holdTimer.reset()

            val errL = ticksToRun - this.odometryEncoders.left.currentPosition
            val errR = ticksToRun - this.odometryEncoders.right.currentPosition

            val base = if(reachedTarget) 0.03 else 0.05
            val mlt = if(reachedTarget) 4.0 else 2.5

            val div = mlt * this.odometryEncoders.ticksPerRev
            val addL =
                if (errL < 0) -base
                else base

            val addR =
                if (errR < 0) -base
                else base

            this.drivetrain.front.left.power = errL / div + addL
            this.drivetrain.front.right.power = errR / div + addR
            this.drivetrain.back.left.power = errL / div + addL
            this.drivetrain.back.right.power = errR / div + addR

            this.telemetry.addLine("Driving...")
            this.telemetry.addData("Tick error left", errL)
            this.telemetry.addData("Tick error right", errR)

            this.telemetry.addData("Front left", "%.2f".format(this.drivetrain.front.left.power))
            this.telemetry.addData("Front right", "%.2f".format(this.drivetrain.front.right.power))
            this.telemetry.addData("Back left", "%.2f".format(this.drivetrain.back.left.power))
            this.telemetry.addData("Back right", "%.2f".format(this.drivetrain.back.right.power))

            this.telemetry.update()
        }

        this.drivetrain.halt()
    }

    override fun run() {
        var position = ItemDetector.Location.NONE

        while (this.checkState() &&
                position == ItemDetector.Location.NONE &&
                timer.milliseconds() < this.recognizeTimeout
        ) {
            telemetry.addLine("Recognition in progress...")
            telemetry.addData(
                    "Time remaining",
                    "%.2f".format((this.recognizeTimeout - timer.milliseconds()) / 1_000.0)
            )
            telemetry.update()

            position = detector.location
        }

        val targetAngle = when (position) {
            ItemDetector.Location.NONE, ItemDetector.Location.CENTER -> 0.0

            ItemDetector.Location.LEFT -> -30.0

            ItemDetector.Location.RIGHT -> 30.0
        }

        this.driveForward(30.0)
        this.turn(targetAngle)
        this.driveForward(60.0)
        this.requestStop()
    }

    private fun checkState(): Boolean {
        UTILS.lockMotor(
            this.slideTilter.motor,
            this.mlt.slideTilterHold,
            0,
            true,
            8192,
            this.drivetrain.back.right
        )

        UTILS.lockMotor(
            this.arm,
            this.mlt.armHold,
            0
        )

        return this.opModeIsActive()
    }
}