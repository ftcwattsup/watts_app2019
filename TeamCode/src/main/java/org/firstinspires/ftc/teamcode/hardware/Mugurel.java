package org.firstinspires.ftc.teamcode.hardware;

import android.text.style.LineHeightSpan;

import com.qualcomm.ftccommon.configuration.EditI2cDevicesActivityAbstract;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Config;

import java.util.ArrayList;
import java.util.List;

import javax.lang.model.type.NullType;

public class Mugurel {
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public LinearOpMode opmode;

    public final double wheelDiameter = 100.0;
    public final double wheelCircumference = wheelDiameter * Math.PI;
    public double ticksPerRevolution;

    public enum IdentifierType {ALL, LEFT_MID, MID_RIGHT}
    public enum AutonomousMoveType { FORWARD, BACKWARD, LEFT, RIGHT, ROTATE }

    public class Collector {
        public DcMotor rotLeft, rotRight, extender;
        public CRServo mat;

        public int rotTicks = 90;
        public double initialPower = 0.33;
        public double newratio = 1.0;
        public boolean lastTicksWithPower = false;

        Collector(DcMotor _rotLeft, DcMotor _rotRight, DcMotor _extend, CRServo _maturique) {
            rotLeft = _rotLeft;
            rotRight = _rotRight;
            extender = _extend;
            mat = _maturique;
            mat.setDirection(DcMotorSimple.Direction.FORWARD);
            rotLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            rotRight.setDirection(DcMotorSimple.Direction.REVERSE);
            extender.setDirection(DcMotorSimple.Direction.FORWARD);
            rotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void afterInitStart() {
            double power = initialPower * newratio;
            rotLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotLeft.setTargetPosition(0);
            rotRight.setTargetPosition(0);
            rotLeft.setPower(power);
            rotRight.setPower(power);

            extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void addTicks(double y) {
            addTicks((int) (y * rotTicks));
        }

        public void addTicks(int ticks) {
            addTicksint(ticks);
            //rotLeft.setTargetPosition(rotLeft.getTargetPosition() + ticks);
            //rotRight.setTargetPosition(rotRight.getTargetPosition() + ticks);
        }

        public void addTicksint(int ticks) {
            if(Math.abs(ticks) < 10)  return;
            if(lastTicksWithPower)
            {
                rotLeft.setPower(initialPower * newratio);
                rotRight.setPower(initialPower * newratio);
            }
            lastTicksWithPower = false;
            rotLeft.setTargetPosition(rotLeft.getTargetPosition() + ticks);
            rotRight.setTargetPosition(rotRight.getTargetPosition() + ticks);
        }

        public void addTicksintWithPower(int ticks, double power) {
            lastTicksWithPower = true;
            rotLeft.setPower(initialPower * power * newratio);
            rotLeft.setPower(initialPower * power * newratio);
            rotLeft.setTargetPosition(rotLeft.getTargetPosition() + ticks);
            rotRight.setTargetPosition(rotRight.getTargetPosition() + ticks);
            /*while(rotLeft.isBusy() && rotRight.isBusy())
            {
                if(!opmode.opModeIsActive())
                {
                    rotLeft.setPower(0);
                    rotRight.setPower(0);
                    return;
                }
            }
            rotLeft.setPower(initialPower);
            rotRight.setPower(initialPower);*/
        }

        public void rotateTicks(int ticks) {
            rotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotLeft.setTargetPosition(ticks);
            rotRight.setTargetPosition(ticks);
            rotLeft.setPower(1.0);
            rotRight.setPower(1.0);
            //while(rotLeft.isBusy() && rotRight.isBusy()) {
            //    if(!opmode.opModeIsActive())    return;
            //}
        }

        public void rotate(double speed) {
            rotLeft.setPower(speed);
            rotRight.setPower(speed);
        }

        public void stopRotation()
        {
            rotLeft.setTargetPosition(rotLeft.getCurrentPosition());
            rotRight.setTargetPosition(rotRight.getCurrentPosition());
        }

        public void extend(double speed) {
            extender.setPower(speed);
        }

        public void collect(int direction) {
            if (direction == 0) {
                mat.setPower(0.0);
                mat.getController().pwmDisable();
            } else if (direction == 1) {
                mat.getController().pwmEnable();
                mat.setPower(1.0);
            } else if(direction == -1) {
                mat.getController().pwmEnable();
                mat.setPower(-1.0);
            } else if(direction == 40) {
                mat.getController().pwmEnable();
                mat.setPower(0.0);
            }
        }
    }

    public class Lifter {
        public DcMotor motor;
        public final static int tickInterval = 6215;

        Lifter(DcMotor _motor) {
            motor = _motor;
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void afterStartInit() {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void move(double speed) {
            if(Math.abs(speed) < 0.1)
            {
                if(motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)  return;
                motor.setPower(speed);
                return;
            }
            if(motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            {
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            motor.setPower(speed);
        }

        public void goToPositionNoWait(int ticks, double power)
        {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(ticks);
            motor.setPower(power);
        }

        public void goToPosition(int ticks, double power) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(ticks);

            while (motor.isBusy()) {
                if (!opmode.opModeIsActive()) {
                    motor.setPower(0);
                    return;
                }

                motor.setPower(power);
            }
            motor.setPower(0.0);

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void land() {
            goToPosition(tickInterval, 1.0);
        }
        public void hook() { goToPosition(-tickInterval, 1.0); }

        public void stay() {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(0);
            motor.setPower(1.0);
        }
    }

    public class MineralIdentifier {
        public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
        public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
        public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
        public static final String VUFORIA_KEY = Config.vuforiaKey;
        public VuforiaLocalizer vuforia;
        public TFObjectDetector tfod;
        public double cameraX = 1280;
        public double cameraY = 720;
        public double middleX = cameraX / 2.0;
        public double TOP = 250;
        public int last = -1;
        public IdentifierType type = IdentifierType.ALL;

        MineralIdentifier() {
            ;
        }

        public void setType(IdentifierType _type) {
            type = _type;
        }
        public void setMid(double x) { middleX = x; }
        public void setTop(double x) { TOP = x; }

        public void initVuforia() {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
        }

        public void initTfod() {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        }

        public void init() {
            initVuforia();
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }
            telemetry.addData("Vuforia intialized!", " ");
            start();
        }

        public void start() {
            tfod.activate();
        }

        public void stop() {
            tfod.shutdown();
        }

        public boolean valid(Recognition r) {
            double ypos = ((r.getTop() + r.getBottom()) / 2.0);
            if (ypos < TOP) return false;
            if(getHeight(r) < 75)   return false;
            return true;
        }

        public List<Recognition> validRecognitions(List<Recognition> rec) {
            ArrayList<Recognition> list = new ArrayList<Recognition>();
            for (Recognition r : rec)
                if (valid(r))
                    list.add(r);
            return list;
        }

        public double getHeight(Recognition rec)
        {
            return ((rec.getBottom() - rec.getTop()));
        }

        public double getPosition(Recognition rec) {
            return ((rec.getLeft() + rec.getRight()) / 2.0);
        }

        public int getGoldMineral() {
            /**
             * -1 = invalid
             * 0 = left
             * 1 = middle
             * 2 = right
             */
            //List<Recognition> updatedRecognitions = tfod.getRecognitions();
            //if(updatedRecognitions == null) return last;
            List<Recognition> recognitions = validRecognitions(tfod.getRecognitions());

            telemetry.addData("Size", recognitions.size());

            if (type == IdentifierType.ALL) {
                int allcnt = recognitions.size(), goldcnt = 0;
                for (Recognition rec : recognitions)
                    if (rec.getLabel().equals(LABEL_GOLD_MINERAL)) goldcnt++;

                if (allcnt != 3 && goldcnt != 1) return -1;

                double goldx = 0, silverx1 = 0, silverx2 = 0;
                for (Recognition rec : recognitions) {
                    if (rec.getLabel().equals(LABEL_GOLD_MINERAL))
                        goldx = getPosition(rec);
                    else if (rec.getLabel().equals(LABEL_SILVER_MINERAL)) {
                        if (silverx1 == 0) silverx1 = getPosition(rec);
                        else silverx2 = getPosition(rec);
                    }
                }

                if (silverx1 > silverx2) {
                    double aux = silverx1;
                    silverx1 = silverx2;
                    silverx2 = aux;
                }

                if (goldx < silverx1) return 0;
                if (goldx < silverx2) return 1;
                return 2;
            } else if (type == IdentifierType.LEFT_MID) {
                int goldcnt = 0;
                for (Recognition rec : recognitions)
                    if (rec.getLabel().equals(LABEL_GOLD_MINERAL)) goldcnt++;
                if (goldcnt == 0) return 2;
                if (goldcnt > 1) return -1;
                for (Recognition rec : recognitions)
                    if (rec.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        double pos = getPosition(rec);
                        if (pos > middleX) return 1;
                        else return 0;
                    }
                return -1;
            } else if (type == IdentifierType.MID_RIGHT) {
                //telemetry.setAutoClear(false);
                int goldcnt = 0;
                for (Recognition rec : recognitions)
                    if (rec.getLabel().equals(LABEL_GOLD_MINERAL))
                    {
                        goldcnt++;
                        //telemetry.addData("Gold height", getHeight(rec));

                    }
                if (goldcnt == 0) return 0;
                if (goldcnt > 1) return -1;
                for (Recognition rec : recognitions)
                    if (rec.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        double pos = getPosition(rec);
                        if (pos > middleX) return 2;
                        else return 1;
                    }
                return -1;
            }
            return -1;
        }

        public int findGold() {
            //telemetry.setAutoClear(false);
            //start();
            //opmode.sleep(1000);
            int count = 10;
            int[] fq = new int[3];
            for (int i = 0; i < count; i++) {
                int where = getGoldMineral();
                telemetry.addData(new Integer(i).toString(), where);
                telemetry.update();
                if (where != -1)
                    fq[where]++;
                last = where;
            }
            telemetry.addData("fq 0", fq[0]);
            telemetry.addData("fq 1", fq[1]);
            telemetry.addData("fq 2", fq[2]);
            telemetry.update();
            int mx = -1, id = -1;
            for (int i = 0; i < 3; i++)
                if (fq[i] > mx) {
                    mx = fq[i];
                    id = i;
                }
            stop();
            return id;
        }
    }

    public class Autonomous {
        public BNO055IMU imu;
        public ModernRoboticsI2cRangeSensor back, left, right;
        public Servo marker;

        public double myAngle;

        public final double markerStart = 1.0;
        public final double markerDown = 0.55;

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

            marker = hardwareMap.get(Servo.class, Config.marker);
            marker.setPosition(markerStart);

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

        public void dropMarker()
        {
            marker.setPosition(markerDown);
        }

        public void land() {
            lift.land();
        }

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

    public Runner runner;
    public Collector collector;
    public MineralIdentifier identifier;
    public Autonomous autonomous;
    public Lifter lift;

    Mugurel(HardwareMap hm) {
        hardwareMap = hm;
        runner = new Runner(
                hm.get(DcMotor.class, Config.leftFront),
                hm.get(DcMotor.class, Config.rightFront),
                hm.get(DcMotor.class, Config.leftBack),
                hm.get(DcMotor.class, Config.rightBack)
        );
        collector = new Collector(
                hm.get(DcMotor.class, Config.rotLeft),
                hm.get(DcMotor.class, Config.rotRight),
                hm.get(DcMotor.class, Config.extend),
                hm.get(CRServo.class, Config.maturique)
        );
        lift = new Lifter(hm.get(DcMotor.class, Config.lift));
        identifier = new MineralIdentifier();
        autonomous = new Autonomous();
    }

    public void initTelemetry(Telemetry _t) {
        telemetry = _t;
        runner.setTelemetry(_t);
    }

    public void setOpmode(LinearOpMode _opmode) {
        opmode = _opmode;
        runner.setOpmode(_opmode);
    }

    public void afterStartInit() {
        runner.afterStartInit();
        collector.afterInitStart();
        lift.afterStartInit();
    }
}




