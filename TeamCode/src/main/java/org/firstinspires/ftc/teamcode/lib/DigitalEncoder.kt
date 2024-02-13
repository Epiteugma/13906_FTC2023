package org.firstinspires.ftc.teamcode.lib

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.commands.core.LynxGetSingleDIOInputCommand
import com.qualcomm.hardware.lynx.commands.core.LynxGetSingleDIOInputResponse
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorControllerEx
import com.qualcomm.robotcore.hardware.DcMotorImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.lib.math.asInt
import java.lang.Exception

class DigitalEncoder(hardwareMap: HardwareMap, nameA: String, nameB: String) : DcMotorImplEx(PlaceboMotorController(), -1) {
    private val hub: LynxModule

    private val channelA: DigitalChannel
    private val channelB: DigitalChannel
    private var position = 0
    private var lastState = 0

    class PlaceboMotorController : DcMotorControllerEx {
        override fun getManufacturer(): HardwareDevice.Manufacturer {
            return HardwareDevice.Manufacturer.Unknown
        }

        override fun getDeviceName(): String {
            return "Digital Encoder"
        }

        override fun getConnectionInfo(): String {
            return "Connected"
        }

        override fun getVersion(): Int {
            return 1
        }

        override fun resetDeviceConfigurationForOpMode(motor: Int) { }

        override fun resetDeviceConfigurationForOpMode() { }

        override fun close() { }

        override fun setMotorType(motor: Int, motorType: MotorConfigurationType?) { }

        override fun getMotorType(motor: Int): MotorConfigurationType {
            return MotorConfigurationType()
        }

        override fun setMotorMode(motor: Int, mode: DcMotor.RunMode?) { }

        override fun getMotorMode(motor: Int): DcMotor.RunMode {
            return DcMotor.RunMode.RUN_USING_ENCODER
        }

        override fun setMotorPower(motor: Int, power: Double) { }

        override fun getMotorPower(motor: Int): Double {
            return 0.0
        }

        override fun isBusy(motor: Int): Boolean {
            return false
        }

        override fun setMotorZeroPowerBehavior(motor: Int, zeroPowerBehavior: DcMotor.ZeroPowerBehavior?) { }

        override fun getMotorZeroPowerBehavior(motor: Int): DcMotor.ZeroPowerBehavior {
            return DcMotor.ZeroPowerBehavior.FLOAT
        }

        override fun getMotorPowerFloat(motor: Int): Boolean {
            return true
        }

        override fun setMotorTargetPosition(motor: Int, position: Int, tolerance: Int) { }

        override fun setMotorTargetPosition(motor: Int, position: Int) { }

        override fun getMotorTargetPosition(motor: Int): Int {
            return 0
        }

        override fun getMotorCurrentPosition(motor: Int): Int {
            return 0
        }

        override fun setMotorEnable(motor: Int) { }

        override fun setMotorDisable(motor: Int) { }

        override fun isMotorEnabled(motor: Int): Boolean {
            return true
        }

        override fun setMotorVelocity(motor: Int, ticksPerSecond: Double) { }

        override fun setMotorVelocity(motor: Int, angularRate: Double, unit: AngleUnit) { }

        override fun getMotorVelocity(motor: Int): Double {
            return 0.0
        }

        override fun getMotorVelocity(motor: Int, unit: AngleUnit): Double {
            return 0.0
        }

        @Deprecated(message = "")
        override fun setPIDCoefficients(motor: Int, mode: DcMotor.RunMode, pidCoefficients: PIDCoefficients) { }

        override fun setPIDFCoefficients(motor: Int, mode: DcMotor.RunMode, pidfCoefficients: PIDFCoefficients) { }

        @Deprecated(message = "", replaceWith = ReplaceWith("PIDCoefficients()", "com.qualcomm.robotcore.hardware.PIDCoefficients"))
        override fun getPIDCoefficients(motor: Int, mode: DcMotor.RunMode): PIDCoefficients {
            return PIDCoefficients()
        }

        override fun getPIDFCoefficients(motor: Int, mode: DcMotor.RunMode?): PIDFCoefficients {
            return PIDFCoefficients()
        }

        override fun getMotorCurrent(motor: Int, unit: CurrentUnit): Double {
            return 0.0
        }

        override fun getMotorCurrentAlert(motor: Int, unit: CurrentUnit): Double {
            return 0.0
        }

        override fun setMotorCurrentAlert(motor: Int, current: Double, unit: CurrentUnit) { }

        override fun isMotorOverCurrent(motor: Int): Boolean {
            return false
        }
    }

    init {
        this.channelA = hardwareMap.get(DigitalChannel::class.java, nameA)
        this.channelB = hardwareMap.get(DigitalChannel::class.java, nameB)

        this.channelA.mode = DigitalChannel.Mode.INPUT
        this.channelB.mode = DigitalChannel.Mode.INPUT

        val hubs = hardwareMap.getAll(LynxModule::class.java)
        this.hub = hubs[0] // TODO: actually pass CH
    }

    private fun poll(port: Int): Int {
        if(port < 0 || port > 7) return 0

        return try {
            val command = LynxGetSingleDIOInputCommand(this.hub, port)
            val response = command.sendReceive() as LynxGetSingleDIOInputResponse
            response.value.asInt()
        } catch (e: Exception) {
            0
        }
    }

    override fun getCurrentPosition(): Int {
        return this.position
    }

    fun update() {
//        val lsb = this.channelA.state.asInt()
//        val msb = this.channelB.state.asInt()
        val lsb = poll(0)
        val msb = poll(1)
        val dir = if(this.direction == DcMotorSimple.Direction.FORWARD) 1 else -1

        val state = (msb shl 1) or lsb
        val sum = (this.lastState shl 2) or state

        if(state == this.lastState) return
        lastState = state

        when(sum) {
            0b1101, 0b0100, 0b0010, 0b1011 -> this.position += 1 * dir
            0b1110, 0b0111, 0b0001, 0b1000 -> this.position -= 1 * dir
        }

        this.lastState = state
    }
}