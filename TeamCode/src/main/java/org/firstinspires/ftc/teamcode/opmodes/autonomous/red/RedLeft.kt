package org.firstinspires.ftc.teamcode.opmodes.autonomous.red

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.AutonomousBase
import org.firstinspires.ftc.teamcode.Color
import org.firstinspires.ftc.teamcode.Side
import org.firstinspires.ftc.teamcode.lib.vision.detection.ItemDetector

@Autonomous(name = "RED LEFT", group = "FTC23")
class RedLeft: AutonomousBase(Color.RED, Side.LEFT) {
    override fun run() {
        super.run()

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

}