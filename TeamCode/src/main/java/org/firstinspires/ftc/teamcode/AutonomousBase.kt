package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.amarcolini.joos.dashboard.JoosConfig
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.lib.UTILS
import org.firstinspires.ftc.teamcode.lib.math.cap
import org.firstinspires.ftc.teamcode.lib.math.normalize
import org.firstinspires.ftc.teamcode.lib.vision.detection.ItemDetector
import org.firstinspires.ftc.teamcode.lib.vision.detection.Prop
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import kotlin.math.abs

enum class Side {
    LEFT,
    RIGHT
}

enum class Color {
    RED,
    BLUE
}

open class AutonomousBase(private val color:Color, private val side:Side) : OpMode() {
    private lateinit var imu: IMU
    private lateinit var detector: ItemDetector
    private val matchTimer = ElapsedTime()
    private val recognizeTimeout = 5_000
    private val turnErrorThreshold:Double  = 5.0
    private val driveErrorThreshold:Double = 2.0
    private val width: Int = 640
    private val height: Int = 480
    var position = ItemDetector.Location.NONE
    private lateinit var webcam: OpenCvCamera
    private val prop: Prop = Prop(color)

    @JoosConfig
    object TurnMlts {
        val base = 0.5
        val baseReached = 0.0
        val mlt = 0.4
        val mltReached = 0.4
    }

    @JoosConfig
    object DriveMlts {
        val base = 0.05
        val baseReached = 0.03
        val mlt = 2.5
        val mltReached = 4.0
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

    fun turn(heading: Double, holdTime: Double = 2.0) {
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

            val base = if(reachedTarget) TurnMlts.baseReached else TurnMlts.base
            val mlt = if(reachedTarget) TurnMlts.mltReached else TurnMlts.mlt

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

    fun driveForward(distance: Double, holdTime: Double = 3.0) {
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

            val base = if(reachedTarget) DriveMlts.baseReached else DriveMlts.base
            val mlt = if(reachedTarget) DriveMlts.mltReached else DriveMlts.mlt

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

    override fun setup() {
        super.setup()

        this.imu = this.hardwareMap.get(IMU::class.java, "imu")

        val imuParams = IMU.Parameters(RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ))

        this.imu.initialize(imuParams)
        this.imu.resetYaw()

        val webcamName: WebcamName = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        val monitorViewIdParent = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        webcam =  OpenCvCameraFactory.getInstance().createWebcam(webcamName, monitorViewIdParent)

        this.waitForStart()
        matchTimer.reset()

        UTILS.resetEncoder(this.arm)
        UTILS.resetEncoder(this.slideTilter.encoder)
    }

    override fun run() {
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

        while (this.checkState() &&
                position == ItemDetector.Location.NONE &&
                matchTimer.milliseconds() < this.recognizeTimeout
        ) {
            telemetry.addLine("Recognition in progress...")
            telemetry.addData(
                    "Time remaining",
                    "%.2f".format((this.recognizeTimeout - matchTimer.milliseconds()) / 1_000.0)
            )
            telemetry.update()

            position = detector.location
        }


    }
}