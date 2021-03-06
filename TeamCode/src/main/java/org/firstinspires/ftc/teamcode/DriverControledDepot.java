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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.hardware.AutoMugurel;
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

@TeleOp(name="Driver Controled Depot", group="Linear Opmode")
//@Disabled
public class DriverControledDepot extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Mugurel robot;
    private MyGamepad gaju, duta;

    private BNO055IMU imu;

    private double finalAngle = 0;
    private double rotOther = 0;
    private boolean rotForward = false, inRotation = false;
    private double lastPower = 0.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        gaju = new MyGamepad(gamepad1);
        duta = new MyGamepad(gamepad2);
        robot = new Mugurel(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setOpmode(this);
        initIMU();
        //waitForStart();
        while (!opModeIsActive()&&!isStopRequested()) { telemetry.addData("Status", "Waiting in Init"); telemetry.update(); }
        runtime.reset();

        robot.afterStartInit();
        robot.runner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.runner.setFace(Math.PI);

        double upAllTicks = 3050;
        double upTicksRotate = 2000;
        double up2TicksRotate = upAllTicks - upTicksRotate;
        double downTicksRotate = -upAllTicks;
        double liftTicks = 5900;
        double dupTicks = 400;
        double ddownTicks = -400;
        robot.collector.extendLander = 0;
        boolean xPress = false, bPress = false, dpadr=false,dpadl=false;
        int matState = 0;
        boolean aPress = false, yPress = false, dupPress = false, ddownPress = false;

        while (opModeIsActive()) {

            gaju.update();
            duta.update();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("X", gaju.getValue(MyGamepad.Axes.LEFT_X));
            telemetry.addData("Y", gaju.getValue(MyGamepad.Axes.LEFT_Y));
            telemetry.addData("R", gaju.getValue(MyGamepad.Axes.RIGHT_X));

            if(gaju.getValue(MyGamepad.Buttons.A) )    robot.runner.setFace(0);
            else if(gaju.getValue(MyGamepad.Buttons.Y) )    robot.runner.setFace(Math.PI);
            else if(gaju.getValue(MyGamepad.Buttons.B) )    robot.runner.setFace(Math.PI / 2.0);
            else if(gaju.getValue(MyGamepad.Buttons.X) )    robot.runner.setFace(-Math.PI / 2.0);

            double modifier = 1.0;
            if(gaju.getValue(MyGamepad.Axes.LEFT_TRIGGER) > 0.3)    modifier = 0.3;
            if(gaju.getValue(MyGamepad.Axes.RIGHT_TRIGGER) > 0.3)   modifier = 0.5;

            double gajux = gaju.getValue(MyGamepad.Axes.LEFT_X);
            double gajuy = gaju.getValue(MyGamepad.Axes.LEFT_Y);
            double gajur = gaju.getValue(MyGamepad.Axes.RIGHT_X);

            /*if(gaju.getValue(MyGamepad.Buttons.DPAD_UP))    robot.runner.move(0, 1, 0, modifier);
            else if(gaju.getValue(MyGamepad.Buttons.DPAD_DOWN)) robot.runner.move(0, -1, 0, modifier);
            else if(gaju.getValue(MyGamepad.Buttons.DPAD_LEFT)) robot.runner.move(-1, 0, 0, modifier);
            else if(gaju.getValue(MyGamepad.Buttons.DPAD_RIGHT))    robot.runner.move(1, 0, 0, modifier);
            else    robot.runner.move(gajux, gajuy, gajur, modifier);*/

            if(Math.abs(gajux + gajuy + gajur) > 0.01) {
                inRotation = false;
                robot.runner.move(gajux, gajuy, gajur, modifier);
            }
            else if(gaju.getValue(MyGamepad.Buttons.DPAD_UP)) {
                fancyRotation(135, 0.17, true);
                //robot.runner.setFaceDegrees(0);
            }
            else if(gaju.getValue(MyGamepad.Buttons.DPAD_DOWN)) {
                fancyRotation(-120, 0.17, false);
                //robot.runner.setFaceDegrees(180);
            }
            else if(!inRotation)
                robot.runner.move(gajux, gajuy, gajur, modifier);
            fancyRotationUpdate();

            if(gaju.getValue(MyGamepad.Buttons.LEFT_BUMPER))    robot.lift.goToPositionNoWait((int)liftTicks, 1.0);
            if(gaju.getValue(MyGamepad.Buttons.RIGHT_BUMPER))   robot.lift.goToPositionNoWait(0, 1.0);

            /** /////////////////////////////////////////////////// DUTA
            /////////////////////////////////////////////////////// DUTA
            /////////////////////////////////////////////////////// DUTA
            /////////////////////////////////////////////////////// DUTA */

            robot.lift.move(duta.getValue(MyGamepad.Axes.RIGHT_Y));

            //robot.collector.noEncoder(duta.getValue(MyGamepad.Axes.LEFT_Y));

            robot.collector.update();

            robot.collector.addTicksGamepad(duta.getValue(MyGamepad.Axes.LEFT_Y));

            if(duta.getRawValue(MyGamepad.Buttons.Y))
            {
                if(!yPress)
                {
                    robot.collector.stopRotation();
                    robot.collector.addTicksWithPower((int)upAllTicks, 0.9, 0.2);
                    //robot.collector.addTicksWithPower((int)up2TicksRotate, 0.4);
                    robot.collector.goToLanderPosition();
                    //1robot.collector.collectFor(400);
                    yPress = true;
                }
            }
            else
                yPress = false;
            if(duta.getRawValue(MyGamepad.Buttons.A))
            {
                if(!aPress)
                {
                    robot.collector.stopRotation();
                    robot.collector.addTicksWithPower((int)downTicksRotate, 0.9, 0.1);
                    robot.collector.closeHolder();
                    //robot.collector.rotateMat();
                    aPress = true;
                }
            }
            else
                aPress = false;

            if(duta.getRawValue(MyGamepad.Buttons.DPAD_UP))
            {
                if(!dupPress)
                {
                    robot.collector.stopRotation();
                    robot.collector.addTicksWithPower((int)dupTicks, 0.5, 0.2);
                    dupPress = true;
                }
            }
            else
                dupPress = false;

            if(duta.getRawValue(MyGamepad.Buttons.DPAD_DOWN))
            {
                if(!ddownPress)
                {
                    robot.collector.stopRotation();
                    robot.collector.addTicksWithPower((int)ddownTicks, 0.5, 0.2);
                    ddownPress = true;
                }
            }
            else
                ddownPress = false;

            robot.collector.update();
            if(duta.getValue(MyGamepad.Axes.LEFT_TRIGGER) > 0.3)
                robot.collector.stopRotation();

            double ext = 0.0;
            if(duta.getValue(MyGamepad.Buttons.LEFT_BUMPER))    ext += -1.0;
            if(duta.getValue(MyGamepad.Buttons.RIGHT_BUMPER))   ext += 1.0;
            robot.collector.extend(ext);

            if(duta.getRawValue(MyGamepad.Buttons.X))
            {
                if(!xPress)
                {
                    if(matState == -1)   matState = 0;
                    else    matState = -1;
                    xPress = true;
                }
            }
            else xPress = false;
            if(duta.getRawValue(MyGamepad.Buttons.B))
            {
                if(!bPress)
                {
                    if(matState == 1)   matState = 0;
                    else    matState = 1;
                    bPress = true;
                }
            }
            else bPress = false;

            if(duta.getValue(MyGamepad.Axes.RIGHT_TRIGGER) > 0.3)   robot.collector.collect(0.5);
            else robot.collector.collect((double)matState * 1.0);
            if(duta.getRawValue(MyGamepad.Buttons.DPAD_RIGHT))
            {
                if(!dpadr)
                {
                    robot.collector.closeHolder();
                    dpadr = true;
                }
            }
            else
                dpadr = false;
            if(duta.getRawValue(MyGamepad.Buttons.DPAD_LEFT))
            {
                if(!dpadl)
                {
                    robot.collector.openHolder();
                    dpadl = true;
                }
            }
            else
                dpadl = false;

            robot.collector.showTelemetry();
            robot.lift.showTelemetry();
            telemetry.update();
        }
    }

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //parameters.loggingEnabled      = true;
        //parameters.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (!imu.isGyroCalibrated() && timer.milliseconds() < 1000) {
            telemetry.addData("Gyro", "Calibrating...");
            telemetry.update();
            if(!opModeIsActive())    return;
        }
        telemetry.addData("Gyro", "Calibrated");
        telemetry.update();
    }

    public double getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void fancyRotation(double angle, double other, boolean forward) {
        inRotation = true;
        finalAngle = getHeading() + angle;
        rotOther = other;
        rotForward = forward;
        lastPower = 0;
    }

    public void fancyRotationUpdate() {
        if(inRotation == false) return;
        double other = rotOther;
        double myAngle = getHeading();
        double accepted = 0.5;
        double needAngle = finalAngle;
        double maxDifference = 0.2;
        double angleDecrease = 18.0 - (1.0 - other) * 5.0;
        double distance = getAngleDistance(myAngle, needAngle);

        if(Math.abs(distance) < 80) {
            if(rotForward)  robot.runner.setFaceDegrees(0);
            else robot.runner.setFaceDegrees(180);
        }

        if (Math.abs(distance) < accepted) {
            inRotation = false;
            robot.runner.reset(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return;
        }

        double power = 0.0;

        if (Math.abs(distance) < angleDecrease)
            power = Math.abs(distance) / angleDecrease;
        else
            power = 1.0;

        power = Math.min(power, lastPower + maxDifference);
        power = Math.max(power, lastPower - maxDifference);
        lastPower = power;
        power = -power;
        if (distance < 0) power = -power;
        //power = getPower(power);

        //runner.angleMove(0, 0, power);
        if(rotForward == true)
        {
            if(power < 0)   robot.runner.fancyRotateMove(-power * other, -power);
            else    robot.runner.fancyRotateMove(power, power * other);
        }
        else
        {
            if(power < 0)   robot.runner.fancyRotateMove(power, power * other);
            else    robot.runner.fancyRotateMove(-power * other, -power);
        }

        if (!robot.opmode.opModeIsActive()) {
            robot.runner.stop();
            return;
        }
    }

    public double getAngleDistance(double start, double fin) {
        start = AngleUnit.normalizeDegrees(start);
        fin = AngleUnit.normalizeDegrees(fin);
        double dist = fin - start;
        dist = AngleUnit.normalizeDegrees(dist);
        return dist;
    }
}

