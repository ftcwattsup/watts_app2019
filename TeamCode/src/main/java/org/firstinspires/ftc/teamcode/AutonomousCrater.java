/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous Crater", group="Linear Opmode")
//@Disabled
public class AutonomousCrater extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Mugurel robot;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        robot = new Mugurel(hardwareMap);
        robot.setOpmode(this);
        robot.initTelemetry(telemetry);
        //robot.identifier.init();
        robot.autonomous.init();
        telemetry.update();

        waitForStart();
        runtime.reset();

        robot.afterStartInit();

        int mineral = 1;
        //mineral = robot.identifier.findGold();

        robot.autonomous.move(150, Math.PI);
        robot.autonomous.rotateP(-90);
        robot.autonomous.move(300, 0);

        if(mineral == 0)    /// Left
            robot.autonomous.move(550, Math.PI / 2);
        else if(mineral == 1)   /// Middle
            robot.autonomous.move(100, Math.PI / 2);
        else if(mineral == 2)   /// Right
            robot.autonomous.move(300, -Math.PI / 2);

        robot.autonomous.move(300, 0);
        robot.autonomous.move(300, Math.PI);

        robot.autonomous.rotateP(90);

        robot.autonomous.moveUntilDistance(450, 0);
        robot.autonomous.rotateP(45);
        sleep(400);
        robot.autonomous.moveUntilDistance(500, 0);

        sleep(1000);

        robot.autonomous.move(1450, Math.PI);

        while(opModeIsActive()) { ; }
    }
}
