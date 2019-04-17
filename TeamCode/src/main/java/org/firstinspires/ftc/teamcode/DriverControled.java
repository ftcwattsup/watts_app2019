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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        robot.setTelemetry(telemetry);
        robot.setOpmode(this);

        //waitForStart();
        while (!opModeIsActive()&&!isStopRequested()) { telemetry.addData("Status", "Waiting in Init"); telemetry.update(); }
        runtime.reset();

        robot.afterStartInit();
        robot.runner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.runner.setFace(Math.PI);

        double newratio = 1.0;
        double upTicksRotate = 950 * newratio;
        double downTicksRotate = -1250 * newratio;
        double dDownTicks = -300 * newratio;
        double landerTicks = 300 * newratio;
        double liftTicks = -5300;
        boolean xPress = false, bPress = false, ddownPress = false, dupPress = false;
        int matState = 0;

        boolean aPress = false, yPress = false;

        while (opModeIsActive()) {

            gaju.update();
            duta.update();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("X", gaju.getValue(MyGamepad.Axes.LEFT_X));
            telemetry.addData("Y", gaju.getValue(MyGamepad.Axes.LEFT_Y));
            telemetry.addData("R", gaju.getValue(MyGamepad.Axes.RIGHT_X));

            if(gaju.getValue(MyGamepad.Buttons.Y) )    robot.runner.setFace(0);
            else if(gaju.getValue(MyGamepad.Buttons.A) )    robot.runner.setFace(Math.PI);
            else if(gaju.getValue(MyGamepad.Buttons.X) )    robot.runner.setFace(Math.PI / 2.0);
            else if(gaju.getValue(MyGamepad.Buttons.B) )    robot.runner.setFace(-Math.PI / 2.0);

            double modifier = 1.0;
            if(gaju.getValue(MyGamepad.Axes.LEFT_TRIGGER) > 0.3)    modifier = 0.3;
            if(gaju.getValue(MyGamepad.Axes.RIGHT_TRIGGER) > 0.3)   modifier = 0.5;

            double gajux = gaju.getValue(MyGamepad.Axes.LEFT_X);
            double gajuy = gaju.getValue(MyGamepad.Axes.LEFT_Y);
            double gajur = gaju.getValue(MyGamepad.Axes.RIGHT_X);

            if(gaju.getValue(MyGamepad.Buttons.DPAD_UP))    robot.runner.move(0, 1, 0, modifier);
            else if(gaju.getValue(MyGamepad.Buttons.DPAD_DOWN)) robot.runner.move(0, -1, 0, modifier);
            else if(gaju.getValue(MyGamepad.Buttons.DPAD_LEFT)) robot.runner.move(-1, 0, 0, modifier);
            else if(gaju.getValue(MyGamepad.Buttons.DPAD_RIGHT))    robot.runner.move(1, 0, 0, modifier);
            else    robot.runner.move(gajux, gajuy, gajur, modifier);

            if(gaju.getValue(MyGamepad.Buttons.LEFT_BUMPER))    robot.lift.goToPositionNoWait((int)liftTicks, 1.0);
            if(gaju.getValue(MyGamepad.Buttons.RIGHT_BUMPER))   robot.lift.goToPositionNoWait(0, 1.0);

            /** /////////////////////////////////////////////////// DUTA
            /////////////////////////////////////////////////////// DUTA
            /////////////////////////////////////////////////////// DUTA
            /////////////////////////////////////////////////////// DUTA */

            robot.lift.move(duta.getValue(MyGamepad.Axes.RIGHT_Y));

            robot.collector.addTicksGamepad(duta.getValue(MyGamepad.Axes.LEFT_Y));

            if(duta.getRawValue(MyGamepad.Buttons.Y))
            {
                if(!yPress)
                {
                    robot.collector.addTicks((int)upTicksRotate);
                    yPress = true;
                }
            }
            else
                yPress = false;
            if(duta.getRawValue(MyGamepad.Buttons.A))
            {
                if(!aPress)
                {
                    robot.collector.addTicksWithPower((int)downTicksRotate, 0.5);
                    aPress = true;
                }
            }
            else
                aPress = false;
            if(duta.getValue(MyGamepad.Buttons.DPAD_DOWN))
            {
                if(!ddownPress)
                {
                    robot.collector.addTicksWithPower((int)dDownTicks, 0.5);
                    ddownPress = true;
                }
            }
            else
                ddownPress = false;

            if(duta.getValue(MyGamepad.Buttons.DPAD_UP))
            {
                if(!dupPress)
                {
                    robot.collector.addTicksWithPower((int)landerTicks, 0.4);
                    dupPress = true;
                }
            }
            else
                dupPress = false;

            if(duta.getValue(MyGamepad.Axes.LEFT_TRIGGER) > 0.3)
                robot.collector.stopRotation();

            double ext = 0.0;
            if(duta.getValue(MyGamepad.Buttons.LEFT_BUMPER))    ext += -1.0;
            if(duta.getValue(MyGamepad.Buttons.RIGHT_BUMPER))   ext += 1.0;
            robot.collector.extend(ext * 0.9);

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

            if(duta.getValue(MyGamepad.Axes.RIGHT_TRIGGER) > 0.3)   robot.collector.collect(0.2);
            else robot.collector.collect((double)matState * 1.0);

            robot.collector.showTelemetry();
            telemetry.update();
        }
    }
}
