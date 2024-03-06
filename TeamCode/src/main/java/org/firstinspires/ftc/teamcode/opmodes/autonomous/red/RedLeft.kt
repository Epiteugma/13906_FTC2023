package org.firstinspires.ftc.teamcode.opmodes.autonomous.red

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.AutonomousBase
import org.firstinspires.ftc.teamcode.Color
import org.firstinspires.ftc.teamcode.Side

@Autonomous(name = "RED LEFT NO PARK", group = "FTC23")
class RedLeft: AutonomousBase(Color.RED, Side.LEFT) {
    override fun run() {
        super.run()

        this.lastPositions.arm = 200
        this.placePixel(position)

        this.requestStop()
    }

}