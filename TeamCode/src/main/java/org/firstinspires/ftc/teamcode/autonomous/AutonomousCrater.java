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

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Mugurel;


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
@Disabled
public class AutonomousCrater extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Mugurel robot;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        double fromMiddle = 120;
        double betweenMinerals = 357;
        double inFrontOfMinerals = 350;
        double scoreMinerals = 320;
        double toWall = 650;
        double toCrater = 1400;

        robot = new Mugurel(hardwareMap);
        robot.setOpmode(this);
        robot.initTelemetry(telemetry);
        robot.identifier.init();
        robot.identifier.setType(Mugurel.IdentifierType.LEFT_MID);
        robot.autonomous.init();
        telemetry.update();

        waitForStart();
        runtime.reset();

        robot.autonomous.land();
        robot.autonomous.moveForwardBackward(fromMiddle, Mugurel.AutonomousMoveType.FORWARD);

        int mineral = 1;
        mineral = robot.identifier.findGold();

        robot.autonomous.rotateTo(-90);
        robot.autonomous.moveForwardBackward(inFrontOfMinerals, Mugurel.AutonomousMoveType.FORWARD);
        robot.autonomous.rotateTo(0);

        double distance = betweenMinerals + fromMiddle;
        if(mineral == 0)    /// Left
        {
            robot.autonomous.moveForwardBackward(betweenMinerals - fromMiddle, Mugurel.AutonomousMoveType.FORWARD);
            distance = 0;
        }
        else if(mineral == 1)   /// Middle
        {
            robot.autonomous.moveForwardBackward(fromMiddle, Mugurel.AutonomousMoveType.BACKWARD);
            distance = betweenMinerals;
        }
        else if(mineral == 2)
        {
            robot.autonomous.moveForwardBackward(fromMiddle + betweenMinerals, Mugurel.AutonomousMoveType.BACKWARD);
            distance = 2 * betweenMinerals;
        }

        /*robot.autonomous.rotateTo(-90);
        robot.autonomous.moveForwardBackward(scoreMinerals, Mugurel.AutonomousMoveType.FORWARD);
        robot.autonomous.moveForwardBackward(scoreMinerals, Mugurel.AutonomousMoveType.BACKWARD);
        robot.autonomous.rotateTo(0);*/

        robot.autonomous.moveLeftRight(scoreMinerals, Mugurel.AutonomousMoveType.RIGHT);
        robot.autonomous.moveLeftRight(scoreMinerals, Mugurel.AutonomousMoveType.LEFT);
        robot.autonomous.rotateTo(0);

        robot.autonomous.moveForwardBackward(toWall + distance, Mugurel.AutonomousMoveType.FORWARD);
        robot.autonomous.rotateTo(-135);
        robot.autonomous.moveSensorDistance(robot.autonomous.left, 150);
        robot.autonomous.rotateTo(-135);

        robot.autonomous.moveSensorDistance(robot.autonomous.back, 400);

        robot.autonomous.dropMarker();
        sleep(200);

        robot.autonomous.moveForwardBackward(toCrater, Mugurel.AutonomousMoveType.FORWARD);

        while(opModeIsActive()) { ; }
    }
}
