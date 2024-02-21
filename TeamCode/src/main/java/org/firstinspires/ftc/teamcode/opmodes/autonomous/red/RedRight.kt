package org.firstinspires.ftc.teamcode.opmodes.autonomous.red

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.AutonomousBase
import org.firstinspires.ftc.teamcode.Color
import org.firstinspires.ftc.teamcode.Side
import org.firstinspires.ftc.teamcode.lib.vision.detection.ItemDetector

@Autonomous(name = "RED RIGHT", group = "FTC23")
class RedRight: AutonomousBase(Color.RED, Side.RIGHT) {
    override fun run() {
        super.run()

        when (position) {
            ItemDetector.Location.NONE, ItemDetector.Location.CENTER -> {
                this.driveForward(60.0)
            }

            ItemDetector.Location.LEFT -> {
                this.turn(-30.0)
                this.driveForward(30.0)
            }

            ItemDetector.Location.RIGHT -> {
                this.turn(30.0)
                this.driveForward(30.0)
            }
        }

        this.requestStop()
    }

}