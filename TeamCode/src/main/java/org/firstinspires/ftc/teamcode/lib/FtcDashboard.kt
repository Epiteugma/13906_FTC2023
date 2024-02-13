package org.firstinspires.ftc.teamcode.lib

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource

object FtcDashboard {

    fun initTelemetry(opMode: LinearOpMode) {
        try {
            val clazz = Class.forName("com.acmerobotics.dashboard.FtcDashboard")
            val getInstance = clazz.getDeclaredMethod("getInstance")

            val instance = getInstance.invoke(null)
            val dashTelemetry = clazz.getMethod("getTelemetry").invoke(instance) as Telemetry

            val telemetryClass = Class.forName("com.acmerobotics.dashboard.telemetry.MultipleTelemetry")

            val newTelemetry = telemetryClass.getConstructor(Array<Telemetry>::class.java).newInstance(arrayOf(opMode.telemetry, dashTelemetry))
            opMode.telemetry = newTelemetry as Telemetry
        } catch(ignored: Exception) { }
    }

    fun initCamera(source: CameraStreamSource, maxFps: Double): Boolean {
        return try {
            val clazz = Class.forName("com.acmerobotics.dashboard.FtcDashboard")
            val getInstance = clazz.getDeclaredMethod("getInstance")

            val instance = getInstance.invoke(null)
            val startCameraStream = clazz.getMethod("startCameraStream", CameraStreamSource::class.java, Double::class.java)

            startCameraStream.invoke(instance, source, maxFps)
            true
        } catch (e: Exception) {
            false
        }
    }

}