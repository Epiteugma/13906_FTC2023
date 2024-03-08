package org.firstinspires.ftc.teamcode.opmodes.autonomous.red

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.AutonomousBase
import org.firstinspires.ftc.teamcode.Color
import org.firstinspires.ftc.teamcode.Side

@Autonomous(name = "RED RIGHT PARK", group = "FTC23")
class RedRight: AutonomousBase(Color.RED, Side.RIGHT) {
    override fun run() {
        super.run()

        this.requestStop()
    }

}