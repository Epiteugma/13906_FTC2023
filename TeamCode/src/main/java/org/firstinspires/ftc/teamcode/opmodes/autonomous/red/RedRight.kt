package org.firstinspires.ftc.teamcode.opmodes.autonomous.red

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.AutonomousBase
import org.firstinspires.ftc.teamcode.Color
import org.firstinspires.ftc.teamcode.Side

@Autonomous(name = "RED RIGHT PARK", group = "FTC23")
class RedRight: AutonomousBase(Color.RED, Side.RIGHT) {
    override fun run() {
        super.run()

        this.lastPositions.arm = 200

        this.placePixel(position)
        this.park(position, -1)

        this.lastPositions.arm = -500
        this.ensureState()
        this.claw.open()

        this.requestStop()
    }

}