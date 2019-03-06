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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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

@TeleOp(name="Driver Controled", group="Linear Opmode")
//@Disabled
public class DriverControled extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Mugurel robot;
    private MyGamepad gaju, duta;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        gaju = new MyGamepad(gamepad1);
        duta = new MyGamepad(gamepad2);
        robot = new Mugurel(hardwareMap);
        robot.initTelemetry(telemetry);
        robot.setOpmode(this);

        waitForStart();
        runtime.reset();

        robot.afterStartInit();
        robot.runner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean xPress = false, bPress = false, aPress = false, yPress = false;
        int matState = 0;

        while (opModeIsActive()) {

            gaju.update();
            duta.update();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("X", gaju.getValue(MyGamepad.Axes.LEFT_X));
            telemetry.addData("Y", gaju.getValue(MyGamepad.Axes.LEFT_Y));
            telemetry.addData("R", gaju.getValue(MyGamepad.Axes.RIGHT_X));

            if(gaju.getValue(MyGamepad.Buttons.Y) == true)    robot.runner.setFace(0);
            else if(gaju.getValue(MyGamepad.Buttons.A) == true)    robot.runner.setFace(Math.PI);
            else if(gaju.getValue(MyGamepad.Buttons.X) == true)    robot.runner.setFace(Math.PI / 2.0);
            else if(gaju.getValue(MyGamepad.Buttons.B) == true)    robot.runner.setFace(-Math.PI / 2.0);

            double modifier = 1.0;
            if(gaju.getValue(MyGamepad.Axes.LEFT_TRIGGER) > 0.3)    modifier = 0.5;
            if(gaju.getValue(MyGamepad.Axes.RIGHT_TRIGGER) > 0.3)   modifier = 0.3;

            double gajux = gaju.getValue(MyGamepad.Axes.LEFT_X);
            double gajuy = gaju.getValue(MyGamepad.Axes.LEFT_Y);
            double gajur = gaju.getValue(MyGamepad.Axes.RIGHT_X);
            //if(gaju.getValue(MyGamepad.Buttons.X))  gajux += -1.0;
            //if(gaju.getValue(MyGamepad.Buttons.B))  gajux += 1.0;

            if(gaju.getValue(MyGamepad.Buttons.DPAD_UP))    robot.runner.move(0, 1, 0, modifier);
            else if(gaju.getValue(MyGamepad.Buttons.DPAD_DOWN)) robot.runner.move(0, -1, 0, modifier);
            else if(gaju.getValue(MyGamepad.Buttons.DPAD_LEFT)) robot.runner.move(-1, 0, 0, modifier);
            else if(gaju.getValue(MyGamepad.Buttons.DPAD_RIGHT))    robot.runner.move(1, 0, 0, modifier);
            else    robot.runner.move(gajux, gajuy, gajur, modifier);

            robot.collector.addTicks(duta.getValue(MyGamepad.Axes.LEFT_Y));

            double ext = 0.0;
            if(duta.getValue(MyGamepad.Buttons.LEFT_BUMPER))    ext += -1.0;
            if(duta.getValue(MyGamepad.Buttons.RIGHT_BUMPER))   ext += 1.0;
            robot.collector.extend(ext * 0.6);
            robot.lift.move(duta.getValue(MyGamepad.Axes.RIGHT_Y));

            if(duta.getRawValue(MyGamepad.Buttons.X))
            {
                if(!xPress)
                {
                    if(matState == 1)   matState = 0;
                    else    matState = 1;
                    xPress = true;
                }
            }
            else xPress = false;
            if(duta.getRawValue(MyGamepad.Buttons.B))
            {
                if(!bPress)
                {
                    if(matState == -1)   matState = 0;
                    else    matState = -1;
                    bPress = true;
                }
            }
            else bPress = false;
            robot.collector.collect(matState);

            if(duta.getRawValue(MyGamepad.Buttons.Y))
            {
                if(!yPress)
                {
                    robot.collector.boxUp();
                    yPress = true;
                }
            }
            else yPress = false;
            if(duta.getRawValue(MyGamepad.Buttons.A))
            {
                if(!aPress)
                {
                    robot.collector.boxDown();
                    aPress = true;
                }
            }
            else aPress = false;

            telemetry.addData("Lift position", robot.lift.motor.getCurrentPosition());
            telemetry.addData("rotLeftTicks", robot.collector.rotLeft.getTargetPosition());
            telemetry.addData("rotRightTicks", robot.collector.rotRight.getTargetPosition());
            telemetry.addData("rotLeftCurrent", robot.collector.rotLeft.getCurrentPosition());
            telemetry.addData("rotRightCurrent", robot.collector.rotRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
