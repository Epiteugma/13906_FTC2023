package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.AutonomousBase
import org.firstinspires.ftc.teamcode.Color
import org.firstinspires.ftc.teamcode.Side

@Autonomous(name = "BLUE RIGHT NO PARK", group = "FTC23")
class BlueRight: AutonomousBase(Color.BLUE, Side.RIGHT) {
    override fun run() {
        super.run()

        this.requestStop()
    }

}