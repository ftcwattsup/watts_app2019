package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class AutoMugurel extends Mugurel {

    public final double wheelDiameter = 100.0;
    public final double wheelCircumference = wheelDiameter * Math.PI;
    public double ticksPerRevolution;

    public enum AutonomousMoveType { FORWARD, BACKWARD, LEFT, RIGHT, ROTATE }

    public class Autonomous {
        public BNO055IMU imu;
        public ModernRoboticsI2cRangeSensor back, left, right;

        public double myAngle;

        public final int rotateMarker = -3800;
        public final int rotateCollect = -4170;
        public final int harmlessPos = -1400;
        public final int parkPos = -3800;
        public final int rotateParkDepot = -3900;

        public int extendLander = 4900;
        public int extendMax = 5400;
        public int preMarkerExtend = 4000;
        public int extendMarker = extendMax;

        public int extendStart = 600;
        public int extendEnd = 3300;

        public double upTicksPre = 400;
        public double upAllTicks = 3150;
        public double upTicksRotate = 2000 - upTicksPre;
        public double up2TicksRotate = upAllTicks - upTicksRotate - upTicksPre;

        Autonomous() {
            ;
        }

        public void init() {
            lift.stay();
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.mode = BNO055IMU.SensorMode.IMU;
            //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            //parameters.loggingEnabled      = true;
            //parameters.loggingTag          = "IMU";
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            myAngle = getHeading();

            back = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, Config.backSensor);
            //back.setI2cAddress(I2cAddr.create8bit(0x28));
            left = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, Config.leftSensor);
            //left.setI2cAddress(I2cAddr.create8bit(0x28));
            right = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, Config.rightSensor);
            //right.setI2cAddress(I2cAddr.create8bit(0x28));

            ticksPerRevolution = runner.leftFront.getMotorType().getTicksPerRev();

            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (!imu.isGyroCalibrated() && timer.milliseconds() < 1000) {
                telemetry.addData("Gyro", "Calibrating...");
                telemetry.update();
                if(!opmode.opModeIsActive())    return;
            }
            telemetry.addData("Gyro", "Calibrated");
            telemetry.update();
        }

        public void harmlessArm() {
            collector.rotateToPosition(harmlessPos, 0.8);
        }

        public void extendForMarker() {
            collector.extendGoToPosition(extendMarker);
            collector.rotateToPosition(rotateMarker, 0.5);
        }

        public void dropMarker() {
            while(collector.rot.isBusy() || collector.extender.isBusy()) { ; }
            collector.collect(-1.0);
            opmode.sleep(500);
            collector.collect(0);
        }

        public void prepareForDepotMarker() {
            collector.rotateToPosition(rotateMarker, 0.5);
            collector.extendGoToPosition(extendMarker);
        }

        public void parkDepot() {
            collector.rotateToPosition(rotateParkDepot, 0.5);
            collector.extendGoToPosition(extendLander);
        }

        public void safeExtend() {
            collector.extendGoToPosition(0);
        }

        public void prepareForExtend() {
            harmlessArm();
            //collector.rotateToPosition(rotateMarker, 0.8);
            collector.extendGoToPosition(preMarkerExtend);
        }

        public void prepareCollect() {
            collector.rotateToPosition(rotateCollect, 0.8);
            collector.extendGoToPosition(extendStart);
        }

        public void collectMineral() {
            collector.collect(1.0);
            collector.extendGoToPositionWait(extendEnd, 1);
            collector.extendGoToPositionWait(extendEnd - 400, 1);
            collector.collect(0);
        }

        public void scoreMineral() {
            //collector.stopRotation();

            collector.rotateToPosition(collector.rot.getCurrentPosition() + (int)upTicksPre, 0.9);
            collector.goToLanderPosition();
            collector.rotateToPosition(collector.rot.getTargetPosition() + (int)upTicksRotate, 0.9);
            rotateTo(0);
            collector.rotateToPositionWait(collector.rot.getTargetPosition() + (int)up2TicksRotate, 0.5);
            collector.openHolder();
            collector.collect(0.5);
            opmode.sleep(400);
            collector.collect(0);
            collector.closeHolder();
        }

        public void makeCycle() {
            collector.collect(1);
            collector.rotateToPosition(collector.rot.getCurrentPosition() - (int)upAllTicks, 0.7);
            moveForwardBackward(150, AutonomousMoveType.FORWARD);

            while(collector.rot.isBusy())   {;}

            collector.extendGoToPositionWait(extendEnd, 1);
            collector.extendGoToPositionWait(extendEnd - 600, 1);

            collector.collect(0);
            collector.extendGoToPosition(extendLander, 1);
            collector.rotateToPosition(collector.rot.getCurrentPosition() + (int)upAllTicks, 0.8);
            moveForwardBackward(150, AutonomousMoveType.BACKWARD);

            collector.openHolder();
            collector.collect(0.5);
            opmode.sleep(400);
            collector.closeHolder();
        }

        public void park() {
            collector.rotateToPosition(parkPos, 0.5);
            moveForwardBackward(300, AutonomousMoveType.FORWARD);
        }

        public void land() {
            lift.land();
        }
        public void hook() { lift.hook(); }

        public void startTracking() {
            imu.startAccelerationIntegration(new Position(), new Velocity(), 200);
        }

        public void stopTracking() {
            imu.stopAccelerationIntegration();
        }

        public void showPosition() {
            Position p = imu.getPosition();
            telemetry.addData("Pos x", p.x);
            telemetry.addData("Pos y", p.y);
            telemetry.addData("Pos z", p.z);
            telemetry.addData("Unit", p.unit.toString());
        }

        public double degToRad(double x) {
            return (x * Math.PI) / 180.0;
        }

        public double radToDeg(double x) {
            return (x * 180.0) / Math.PI;
        }

        public Orientation getGyro() {
            return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }

        public double getHeading() {
            return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }

        public double getHeadingRadians() {
            return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        }

        public double getAngleDistance(double start, double fin) {
            start = AngleUnit.normalizeDegrees(start);
            fin = AngleUnit.normalizeDegrees(fin);
            double dist = fin - start;
            dist = AngleUnit.normalizeDegrees(dist);
            return dist;
        }

        public void showDistances()
        {
            telemetry.addData("Back", back.getDistance(DistanceUnit.MM));
            telemetry.addData("Left", left.getDistance(DistanceUnit.MM));
            telemetry.addData("Right", right.getDistance(DistanceUnit.MM));
            //telemetry.update();
        }

        public double getPower(double pw) {
            return pw;
        }

        public void rotateTo(double degrees) {
            degrees = AngleUnit.normalizeDegrees(degrees);
            double dist = getAngleDistance(getHeading(), degrees);
            rotateP(dist);
        }

        public void rotateToFancy(double degrees, double other, AutonomousMoveType type) {
            degrees = AngleUnit.normalizeDegrees(degrees);
            double dist = getAngleDistance(getHeading(), degrees);
            rotatePFancy(dist, other, type);
        }

        public void rotateP(double degrees) {
            runner.reset(DcMotor.RunMode.RUN_USING_ENCODER);
            double accepted = 0.5;
            double needAngle = AngleUnit.normalizeDegrees(getHeading() + degrees);
            double lastPower = 0.0;
            double maxDifference = 0.1;
            double angleDecrease = 40.0;
            while (true) {
                double myAngle = getHeading();
                telemetry.addData("Heading", myAngle);
                telemetry.addData("Power", lastPower);
                double distance = getAngleDistance(myAngle, needAngle);
                telemetry.addData("Distance", distance);
                telemetry.update();
                if (Math.abs(distance) < accepted) break;

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
                power = getPower(power);

                runner.angleMove(0, 0, power);

                if (!opmode.opModeIsActive()) {
                    runner.stop();
                    return;
                }
            }
            runner.stop();
        }

        public void rotatePFancy(double degrees, double other, AutonomousMoveType type) {
            runner.reset(DcMotor.RunMode.RUN_USING_ENCODER);
            double accepted = 0.5;
            double needAngle = AngleUnit.normalizeDegrees(getHeading() + degrees);
            double lastPower = 0.0;
            double maxDifference = 0.1;
            double angleDecrease = 40.0 - (1.0 - other) * 5.0;
            while (true) {
                double myAngle = getHeading();
                telemetry.addData("Heading", myAngle);
                telemetry.addData("Power", lastPower);
                double distance = getAngleDistance(myAngle, needAngle);
                telemetry.addData("Distance", distance);
                telemetry.update();
                if (Math.abs(distance) < accepted) break;

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
                power = getPower(power);

                //runner.angleMove(0, 0, power);
                if(type == AutonomousMoveType.FORWARD)
                {
                    if(power < 0)   runner.fancyRotateMove(-power * other, -power);
                    else    runner.fancyRotateMove(power, power * other);
                }
                else if(type == AutonomousMoveType.BACKWARD)
                {
                    if(power < 0)   runner.fancyRotateMove(power, power * other);
                    else    runner.fancyRotateMove(-power * other, -power);
                }

                if (!opmode.opModeIsActive()) {
                    runner.stop();
                    return;
                }
            }
            runner.stop();
        }

        public int distanceToTicks(double dist) {
            double ans = (dist * ticksPerRevolution) / wheelCircumference;
            return (int) ans;
        }

        public int distanceToTicksLeftRight(double dist) {
            //double ans = (dist * 1.85); //mm
            double ans = (double) distanceToTicks(dist) * 1.00;
            return (int) ans;
        }

        public void moveForwardBackward(double distance, AutonomousMoveType type)
        {
            if(type != AutonomousMoveType.FORWARD && type != AutonomousMoveType.BACKWARD)   return;
            int ticks = distanceToTicks(distance);
            if(type == AutonomousMoveType.BACKWARD)    ticks = -ticks;
            runner.reset(DcMotor.RunMode.RUN_TO_POSITION);
            runner.setTargetPositions(ticks, ticks, ticks, ticks);
            move();
        }

        public void moveLeftRight(double distance, AutonomousMoveType type)
        {
            if(type != AutonomousMoveType.LEFT && type != AutonomousMoveType.RIGHT) return;
            int ticks = distanceToTicksLeftRight(distance);
            if(type == AutonomousMoveType.LEFT) ticks = -ticks;
            runner.reset(DcMotor.RunMode.RUN_TO_POSITION);
            runner.setTargetPositions(ticks, -ticks, -ticks, ticks);
            double startAngle = getHeading();
            move();
            //rotateTo(startAngle);
        }

        public void move()
        {
            double lastPower = 0.0;
            double maxDifference = 0.1;
            int ticksDecrease = (int) (1.0 * ticksPerRevolution);
            double angleStart = getHeading();
            while (runner.isBusy()) {
                double power = 0.0;

                int ticksDistance = runner.getTicksDistance();

                if (ticksDistance < ticksDecrease)
                    power = (double) ticksDistance / (double) ticksDecrease;
                else
                    power = 1.0;

                power = Math.min(power, lastPower + maxDifference);
                power = Math.max(power, lastPower - maxDifference);
                power = Math.max(power, 0.05);
                lastPower = power;

                runner.setPower(power);
                runner.showPositions();

                if (!opmode.opModeIsActive()) {
                    runner.stop();
                    return;
                }
            }
            runner.stop();
            //rotateTo(angleStart);
        }

        /*public void moveLeftRight(double distance) {
            runner.reset(DcMotor.RunMode.RUN_USING_ENCODER);
            //runner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int numberOfTicks = distanceToTicksLeftRight(distance);
            runner.leftFront.setTargetPosition(-numberOfTicks);
            runner.leftBack.setTargetPosition(numberOfTicks);
            runner.rightFront.setTargetPosition(numberOfTicks);
            runner.rightBack.setTargetPosition(-numberOfTicks);
            runner.leftFront.setPower(0.5);
            runner.rightBack.setPower(0.5);
            runner.rightFront.setPower(0.5);
            runner.leftFront.setPower(0.5);
            while (runner.isBusy()) {
            }
            runner.setPower(0.0);
        }


        public void moveStraight(double distance, double angle) {
            double dx = distance * Math.cos(angle + Math.PI / 2);
            double dy = distance * Math.sin(angle + Math.PI / 2);

            telemetry.addData("dx", dx);
            telemetry.addData("dy", dy);
            telemetry.update();

            int lf = distanceToTicks(dy) + distanceToTicks(dx);
            int rf = distanceToTicks(dy) - distanceToTicks(dx);

            runner.reset(DcMotor.RunMode.RUN_TO_POSITION);
            runner.setTargetPositions(lf, rf, rf, lf);

            double lastPower = 0.0;
            double maxDifference = 0.05;
            int ticksDecrease = (int) (2.5 * ticksPerRevolution);
            double angleDecrease = 45.0;

            double heading = getHeading();

            while (runner.isBusy()) {
                double power = 0.0;

                int ticksDistance = runner.getTicksDistance();

                if (ticksDistance < ticksDecrease)
                    power = (double) ticksDistance / (double) ticksDecrease;
                else
                    power = 1.0;

                power = Math.min(power, lastPower + maxDifference);
                power = Math.max(power, lastPower - maxDifference);
                lastPower = power;
                power = Math.max(power, 0.05);

                double angleDistance = getAngleDistance(getHeading(), heading);
                double rot = 0.0;
                if (Math.abs(angleDistance) > angleDecrease) {
                    if (angleDistance < 0) rot = 1.0;
                    else rot = -1.0;
                } else
                    rot = -(angleDistance / angleDecrease);

                Runner.MotorPowers pw = runner.angleDrive(power, angle, rot);
                runner.setPower(pw);

                runner.showPositions();

                if (!opmode.opModeIsActive()) {
                    runner.stop();
                    return;
                }
            }
            runner.stop();
        }

        public void move(double distance, double angle) {

            angle += Math.PI / 2.0;
            double dx = distance * Math.cos(angle);
            double dy = distance * Math.sin(angle);

            telemetry.addData("dx", dx);
            telemetry.addData("dy", dy);
            telemetry.update();

            int lf = distanceToTicks(dy) + distanceToTicks(dx);
            int rf = distanceToTicks(dy) - distanceToTicks(dx);

            runner.reset(DcMotor.RunMode.RUN_TO_POSITION);
            runner.setTargetPositions(lf, rf, rf, lf);

            double lastPower = 0.0;
            double maxDifference = 0.05;
            int ticksDecrease = (int) (1.25 * ticksPerRevolution);

            while (runner.isBusy()) {
                double power = 0.0;

                int ticksDistance = runner.getTicksDistance();

                if (ticksDistance < ticksDecrease)
                    power = (double) ticksDistance / (double) ticksDecrease;
                else
                    power = 1.0;

                power = Math.min(power, lastPower + maxDifference);
                power = Math.max(power, lastPower - maxDifference);
                lastPower = power;
                power = Math.max(power, 0.05);

                Runner.MotorPowers pw = runner.angleDrive(power, angle, 0);
                runner.setPower(pw);

                runner.showPositions();

                if (!opmode.opModeIsActive()) {
                    runner.stop();
                    return;
                }
            }
            runner.stop();
        }*/

        public void moveSensorDistance(ModernRoboticsI2cRangeSensor sensor, double distance)
        {
            runner.reset(DcMotor.RunMode.RUN_USING_ENCODER);

            int dir = 0;
            if(sensor == back) dir = 0;
            if(sensor == left)  dir = 1;
            if(sensor == right) dir = 2;

            double lastPower = 0.0;
            double maxDifference = 0.05;
            double decrease = 175;
            while (runner.isBusy()) {
                showDistances();
                double now = sensor.getDistance(DistanceUnit.MM);
                if(now <= distance) break;

                double power = 0.0;
                double dif = now - distance;
                if(dif < decrease)
                    power = dif / decrease;
                else
                    power = 1.0;

                power = Math.min(power, lastPower + maxDifference);
                power = Math.max(power, lastPower - maxDifference);
                power = Math.max(power, 0.05);
                lastPower = power;
                telemetry.addData("Power", power);
                telemetry.update();

                if(dir == 0)    runner.setPower(-power);
                if(dir == 1)    runner.setPower(-power, power, power, -power);
                if(dir == 2)    runner.setPower(power, -power, -power, power);

                if (!opmode.opModeIsActive()) {
                    runner.stop();
                    return;
                }
            }
            runner.stop();
        }

        /*public void moveUntilDistance(double distance, double angle) {
            angle += Math.PI / 2.0;
            runner.reset(DcMotor.RunMode.RUN_USING_ENCODER);
            double accepted = 5.0;
            double lastPower = 0.0;
            double maxDifference = 0.1;
            double distanceDecrease = 500.0;
            double maxPower = 0.85;
            while (true) {
                double myDistance = getFrontDistance();
                telemetry.addData("Distance", myDistance);
                telemetry.addData("Power", lastPower);
                double rem = myDistance - distance;
                telemetry.addData("Remaining", rem);
                telemetry.update();
                if (rem < accepted) break;

                double power = 0.0;
                if (rem < distanceDecrease)
                    power = rem / distanceDecrease;
                else
                    power = 1.0;

                power = Math.min(power, lastPower + maxDifference);
                power = Math.max(power, lastPower - maxDifference);
                lastPower = power;

                runner.angleMove(power * maxPower, angle, 0);

                if (!opmode.opModeIsActive()) {
                    runner.stop();
                    return;
                }
            }
            runner.stop();
        }*/
    }

    public MineralIdentifier identifier;
    public Autonomous autonomous;

    public AutoMugurel(HardwareMap hm) {
        hardwareMap = hm;
        runner = new Runner(
                hm.get(DcMotor.class, Config.leftFront),
                hm.get(DcMotor.class, Config.rightFront),
                hm.get(DcMotor.class, Config.leftBack),
                hm.get(DcMotor.class, Config.rightBack)
        );
        collector = new Collector(
                hm.get(DcMotor.class, Config.rot),
                hm.get(DcMotor.class, Config.extend),
                hm.get(DcMotor.class, Config.maturique),
                hm.get(Servo.class, Config.holder)
        );
        lift = new Lift(hm.get(DcMotor.class, Config.lift));
        identifier = new MineralIdentifier(hm);
        autonomous = new Autonomous();
        identifier.init();
        autonomous.init();
    }

    public AutoMugurel(HardwareMap hm, Telemetry _t, LinearOpMode _o) {
        hardwareMap = hm;
        runner = new Runner(
                hm.get(DcMotor.class, Config.leftFront),
                hm.get(DcMotor.class, Config.rightFront),
                hm.get(DcMotor.class, Config.leftBack),
                hm.get(DcMotor.class, Config.rightBack)
        );
        collector = new Collector(
                hm.get(DcMotor.class, Config.rot),
                hm.get(DcMotor.class, Config.extend),
                hm.get(DcMotor.class, Config.maturique),
                hm.get(Servo.class, Config.holder)
        );
        lift = new Lift(hm.get(DcMotor.class, Config.lift));
        identifier = new MineralIdentifier(hm);
        autonomous = new Autonomous();

        setTelemetry(_t);
        setOpmode(_o);

        identifier.init();
        autonomous.init();
    }

    public void setTelemetry(Telemetry _t) {
        telemetry = _t;
        runner.setTelemetry(_t);
        collector.setTelemetry(_t);
        lift.setTelemetry(_t);
        identifier.setTelemetry(_t);
    }

    public void setOpmode(LinearOpMode _o) {
        opmode = _o;
        runner.setOpmode(_o);
        collector.setOpmode(_o);
        lift.setOpmode(_o);
        identifier.setOpmode(_o);
    }
}
