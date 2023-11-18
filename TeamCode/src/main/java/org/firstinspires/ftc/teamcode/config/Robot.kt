package org.firstinspires.ftc.teamcode.config

import com.qualcomm.robotcore.hardware.DcMotor.*
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.*
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

object Robot {
    val wheelRadius = 6.5
    val ticksPerRev = 28
    val driveGearbox = 20.0

    val driveMlt = 0.5

    private val motorNames = arrayListOf("frontLeft", "backLeft", "frontRight", "backRight")
    val motors = mutableMapOf<String, DcMotorEx>()
    val drivetrainLeft = 0..1
    val drivetrainRight = 2..3

    private val servoNames = arrayListOf<String>()
    val servos = mutableMapOf<String, Servo>()

    private val cameraName = "Webcam 1"
    lateinit var camera: WebcamName

    fun initHardware(hardwareMap: HardwareMap) {
        for(i in 0..<this.motorNames.size) {
            val name = this.motorNames[i]
            this.motors[name] = hardwareMap.get(DcMotorEx::class.java, name)

            if(drivetrainLeft.contains(i) || drivetrainRight.contains(i)) {
                this.motors[name]!!.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
                this.motors[name]!!.mode = RunMode.STOP_AND_RESET_ENCODER
                this.motors[name]!!.mode = RunMode.RUN_USING_ENCODER
            }

            if(drivetrainRight.contains(i)) this.motors[name]!!.direction = Direction.REVERSE
        }
        for(name in this.servoNames) this.servos[name] = hardwareMap.get(Servo::class.java, name)

        this.camera = hardwareMap.get(WebcamName::class.java, this.cameraName)
    }

    fun bindDashboard(telemetry: Telemetry): Telemetry {
        try {
            val clazz = Class.forName("com.acmerobotics.dashboard.FtcDashboard")
            val getInstance = clazz.getDeclaredMethod("getInstance")

            val instance = getInstance.invoke(null)
            val dashTelemetry = clazz.getMethod("getTelemetry").invoke(instance) as Telemetry

            val telemetryClass = Class.forName("com.acmerobotics.dashboard.telemetry.MultipleTelemetry")

            val newTelemetry = telemetryClass.getConstructor(Array<Telemetry>::class.java).newInstance(arrayOf(telemetry, dashTelemetry))
            return newTelemetry as Telemetry
        } catch(e: Exception) {
            return telemetry
        }
    }

    fun initDashboardStream(source: AprilTagProcessor, maxFPS: Double): Boolean {
        try {
            val clazz = Class.forName("com.acmerobotics.dashboard.FtcDashboard")
            val getInstance = clazz.getDeclaredMethod("getInstance")

            val instance = getInstance.invoke(null)
            val startCameraStream = clazz.getMethod("startCameraStream", CameraStreamSource::class.java, Double::class.java)

            startCameraStream.invoke(instance, source, maxFPS)
            return true
        } catch (e: Exception) {
            return false
        }
    }

    fun cmToTicks(cm: Double): Int {
        return (cm / (this.wheelRadius * Math.PI * 2) * (this.ticksPerRev * this.driveGearbox)).toInt()
    }

    fun ticksToCM(ticks: Int): Double {
        return ticks / (this.ticksPerRev * this.driveGearbox) * (this.wheelRadius * Math.PI * 2)
    }
}