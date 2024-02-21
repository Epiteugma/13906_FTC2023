package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.AutonomousBase
import org.firstinspires.ftc.teamcode.Color
import org.firstinspires.ftc.teamcode.Side
import org.firstinspires.ftc.teamcode.lib.UTILS
import org.firstinspires.ftc.teamcode.lib.vision.detection.ItemDetector
import kotlin.math.abs

@Autonomous(name = "BLUE LEFT", group = "FTC23")
class RedLeft: AutonomousBase(Color.RED, Side.LEFT) {
    override fun run() {
        super.run()

        this.lastPositions.arm = 200
        when (position) {
            ItemDetector.Location.NONE, ItemDetector.Location.CENTER -> {
                this.driveForward(73.0)
                this.driveForward(-60.0)
                this.turn(90.0)
                this.driveForward(70.0)
            }

            ItemDetector.Location.LEFT -> {
                this.driveForward(25.0)
                this.turn(45.0, true)
//                this.driveForward(10.0)
                this.driveForward(-20.0)
                this.turn(0.0)
                this.driveForward(-30.0)
                this.turn(90.0)
                this.driveForward(70.0)
            }

            ItemDetector.Location.RIGHT -> {
                this.driveForward(25.0)
                this.turn(-45.0, true)
//                this.driveForward(10.0)
                this.driveForward(-20.0)
                this.turn(0.0)
                this.driveForward(-30.0)
                this.turn(90.0)
                this.driveForward(70.0)
            }
        }
        this.lastPositions.arm = -500
        this.ensureState()
        this.claw.open()

        this.requestStop()
    }

}