package org.firstinspires.ftc.teamcode.config

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName

object Robot {
    val wheelRadius = 0.0

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

            if(drivetrainLeft.contains(i) || drivetrainRight.contains(i)) this.motors[name]?.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        }
        for(name in this.servoNames) this.servos[name] = hardwareMap.get(Servo::class.java, name)

        this.camera = hardwareMap.get(WebcamName::class.java, this.cameraName)
    }
}