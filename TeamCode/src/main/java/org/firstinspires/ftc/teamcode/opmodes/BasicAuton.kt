package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.OpMode
import org.firstinspires.ftc.teamcode.lib.UTILS
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
    private val driveErrorThreshold = 2

    private lateinit var leftEncoder: DcMotor
    private lateinit var rightEncoder: DcMotor

    override fun setup() {
        super.setup()

        this.leftEncoder = this.drivetrain.front.left
        this.rightEncoder = this.drivetrain.front.right

        this.imu = this.hardwareMap.get(IMU::class.java, "imu")

        val imuParams = IMU.Parameters(RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ))

        this.imu.initialize(imuParams)
        this.imu.resetYaw()

//        this.vision = Vision()
//        this.vision.init(
//                Vision.LensIntrinsics(),
//                2f,
//                this.hardwareMap.get(WebcamName::class.java, "Webcam 1"),
//                mapOf()
//        )

        this.waitForStart()
        timer.reset()

        UTILS.resetEncoder(this.arm)
        UTILS.resetEncoder(this.drivetrain.back.right)
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

    private fun driveForward(distance: Double, holdTime: Double = 5.0) {
        UTILS.resetEncoder(this.leftEncoder)
        UTILS.resetEncoder(this.rightEncoder)

        val tpr = 8192
        val wheelRadius = 6.0 / 2.0

        val circumference = 2 * Math.PI * wheelRadius
        val ticksToRun = distance / circumference * tpr
        val errorThreshold = this.driveErrorThreshold / circumference * tpr
        var reachedTarget = false
        val holdTimer = ElapsedTime()

        while (
            this.checkState() &&
            (!reachedTarget || holdTimer.milliseconds() < holdTime * 1000)
        ) {
            reachedTarget = reachedTarget || (abs(ticksToRun - this.leftEncoder.currentPosition) < errorThreshold &&
                    abs(ticksToRun - this.rightEncoder.currentPosition) < errorThreshold)
            if (!reachedTarget) holdTimer.reset()

            val errL = ticksToRun - this.leftEncoder.currentPosition
            val errR = ticksToRun - this.rightEncoder.currentPosition

            val base = if(reachedTarget) 0.03 else 0.05
            val mlt = if(reachedTarget) 4.0 else 2.5

            val div = mlt * tpr
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
        var position = PropPosition.NONE

        while (this.checkState() &&
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

        val driveDistance = 60.0
        val targetAngle = when (position) {
            PropPosition.NONE, PropPosition.CENTER -> 0.0

            PropPosition.LEFT -> -30.0

            PropPosition.RIGHT -> 30.0
        }

        this.driveForward(30.0, 2.0)
        this.turn(targetAngle)
        this.driveForward(driveDistance)
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