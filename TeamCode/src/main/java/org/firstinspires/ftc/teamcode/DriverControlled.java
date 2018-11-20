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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="Driver Control", group="Linear Opmode")
//@Disabled
public class DriverControlled extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Mugurel robot;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot = new Mugurel(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        robot.afterStartInit();

        boolean g2b = false, g2x = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            /// Gamepad 1
            if(gamepad1.dpad_up)    robot.runner.facing(Mugurel.Face.FRONT);
            if(gamepad1.dpad_down)  robot.runner.facing(Mugurel.Face.BACK);

            if(gamepad1.left_trigger >= 0.5) robot.runner.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 0.25);
            else if(gamepad1.right_trigger >= 0.5) robot.runner.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 0.5);
            else robot.runner.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            /// Gamepad 2

            if(gamepad2.a)  robot.collector.setRotationPower(-gamepad2.left_stick_y * 0.7);
            else robot.collector.setRotationPower(-gamepad2.left_stick_y * 0.3);

            if(gamepad2.left_bumper)    robot.collector.extend(Mugurel.ExtenderDirection.CONTRACT);
            else if(gamepad2.right_bumper)  robot.collector.extend(Mugurel.ExtenderDirection.EXTEND);
            else    robot.collector.extend(Mugurel.ExtenderDirection.STOP);

            if(gamepad2.x)
            {
                if(!g2x)
                {
                    robot.collector.collectorPress(Mugurel.CollectionType.COLLECT);
                    g2x = true;
                }
            }
            else g2x = false;

            if(gamepad2.b)
            {
                if(!g2b)
                {
                    robot.collector.collectorPress(Mugurel.CollectionType.SPIT);
                    g2b = true;
                }
            }
            else
                g2b = false;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}