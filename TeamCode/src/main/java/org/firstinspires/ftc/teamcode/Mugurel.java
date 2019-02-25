package org.firstinspires.ftc.teamcode;

import android.text.style.LineHeightSpan;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

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

import java.util.ArrayList;
import java.util.List;

import javax.lang.model.type.NullType;

public class Mugurel
{
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public LinearOpMode opmode;

    public final double wheelDiameter = 100.0;
    public final double wheelCircumference = wheelDiameter * Math.PI;
    public double ticksPerRevolution;

    public enum IdentifierType { ALL, LEFT_MID, MID_RIGHT };

    public class Runner
    {
        /**
         * Motor functions
         */
        public class MotorPowers
        {
            public double lf, lb, rf, rb;
            MotorPowers() { lf = lb = rf = rb = 0; }
            MotorPowers(double _lf, double _lb, double _rf, double _rb) { lf = _lf; lb = _lb; rf = _rf; rb = _rb; }

            public void normalize()
            {
                double mx = Math.max( Math.max(Math.abs(lf), Math.abs(lb)), Math.max(Math.abs(rf), Math.abs(rb)));
                if(mx > 1.0)
                {
                    lf /= mx;
                    lb /= mx;
                    rf /= mx;
                    rb /= mx;
                }
            }

            public void speed(double spd)
            {
                spd = Math.abs(spd);

                if(spd > 1.0)   spd = 1.0;
                double mx = Math.max( Math.max(Math.abs(lf), Math.abs(lb)), Math.max(Math.abs(rf), Math.abs(rb)) );
                if(spd <= 1.0 && mx < spd)
                {
                    double coef = spd / mx;
                    lf *= coef;
                    lb *= coef;
                    rf *= coef;
                    rb *= coef;
                }
            }

            public void rap(double r)
            {
                lf *= r; lb *= r; rf *= r; rb *= r;
            }
        }
        public DcMotor leftFront, rightFront, leftBack, rightBack;
        public double faceAngle;
        public final double wheelAngle = Math.PI / 4.0;

        Runner(DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb)
        {
            leftFront = lf; rightFront = rf; leftBack = lb; rightBack = rb;
            faceAngle = 0;
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            ticksPerRevolution = leftFront.getMotorType().getTicksPerRev();
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void afterStartInit()
        {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void setMode(DcMotor.RunMode mode) { leftFront.setMode(mode); leftBack.setMode(mode); rightFront.setMode(mode); rightBack.setMode(mode); }

        public void setPower(MotorPowers pw) { setPower(pw.lf, pw.lb, pw.rf, pw.rb); }
        public void setPower(double pw) { setPower(pw, pw, pw, pw); }
        public void setPower(double lf, double lb, double rf, double rb)
        {
            leftFront.setPower(lf);
            leftBack.setPower(lb);
            rightFront.setPower(rf);
            rightBack.setPower(rb);
        }

        /**
         * Move + Drive
         */
        /*public MotorPowers basicDrive(double x, double y, double r)
        {
            double pw[] = new double[4];
            pw[0] = x + y + r;
            pw[1] = -x + y - r;
            pw[2] = -x + y + r;
            pw[3] = x + y - r;

            MotorPowers mpw = new MotorPowers(pw[0], pw[2], pw[1], pw[3]);
            mpw.normalize();
            return mpw;
        }*/

        public MotorPowers angleDriveFromAxes(double x, double y, double r)
        {
            double angle = Math.atan2(y, x);
            telemetry.addData("Angle", angle);
            while(angle < 0)   angle += 2 * Math.PI;
            double speed = Math.sqrt(x * x + y * y);
            telemetry.addData("Speed", speed);
            return angleDrive(speed, angle, r);
        }

        public MotorPowers angleDrive(double speed, double angle, double rot)
        {
            angle += faceAngle;
            angle += wheelAngle;

            double lf = speed * Math.sin(-angle) - rot;
            double rf = speed * Math.cos(-angle) + rot;
            double lb = speed * Math.cos(-angle) - rot;
            double rb = speed * Math.sin(-angle) + rot;
            lf = -lf; rf = -rf; lb = -lb; rb = -rb;

            MotorPowers mpw = new MotorPowers(lf, lb, rf, rb);

            mpw.normalize();
            mpw.speed(speed);
            return mpw;
        }

        public void move(double x, double y, double r) { move(x, y, r, 1.0);}
        public void move(double x, double y, double r, double rap)
        {
            MotorPowers pw = angleDriveFromAxes(x, y, r);
            pw.rap(rap);
            setPower(pw);

            /*telemetry.addData("Face", faceAngle);

            telemetry.addData("Dir lf", leftFront.getDirection().toString());
            telemetry.addData("Dir rf", rightFront.getDirection().toString());
            telemetry.addData("Dir lb", leftBack.getDirection().toString());
            telemetry.addData("Dir rb", rightBack.getDirection().toString());

            telemetry.addData("Front Left", leftFront.getPower());
            telemetry.addData("Front Right", rightFront.getPower());
            telemetry.addData("Back Left", leftBack.getPower());
            telemetry.addData("Back Right", rightBack. getPower());*/

            telemetry.addData("Encoder lf", leftFront.getCurrentPosition());
            telemetry.addData("Encoder rf", rightFront.getCurrentPosition());
            telemetry.addData("Encoder lb", leftBack.getCurrentPosition());
            telemetry.addData("Encoder rb", rightBack.getCurrentPosition());
        }

        public void angleMove(double speed, double angle, double rot) { angleMove(speed, angle, rot, 1.0); }
        public void angleMove(double speed, double angle, double rot, double rap)
        {
            MotorPowers pw = angleDrive(speed, angle, rot);
            pw.rap(rap);
            setPower(pw);
        }

        public void moveTank(double x, double y, double r)
        {
            double left = y + r, right = y - r;
            if( Math.abs(r) < 0.001 )   setPower(left, left ,right, right);
            else if( Math.abs(y) < 0.001 ) setPower(left, left, right, right);
            else
            {
                if(left < -1.0) left = -1.0;
                if(left > 1.0)  left = 1.0;
                if(right < -1.0)    right = -1.0;
                if(right > 1.0) right = 1.0;
                setPower(left, left, right, right);
            }
        }

        public void setFace(double angle) { faceAngle = angle; }

        public void reset(DcMotor.RunMode mode)
        {
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMode(mode);
        }
        public void reset() { reset(leftFront.getMode()); }
        public void stop() { setPower(0); setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }

        public void setTargetPositions(int lf, int lb, int rf, int rb)
        {
            leftFront.setTargetPosition(lf);
            leftBack.setTargetPosition(lb);
            rightFront.setTargetPosition(rf);
            rightBack.setTargetPosition(rb);
        }

        public int getTicksDistance()
        {
            int sum = 0;
            sum += Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition());
            sum += Math.abs(leftBack.getCurrentPosition() - leftBack.getTargetPosition());
            sum += Math.abs(rightFront.getCurrentPosition() - rightFront.getTargetPosition());
            sum += Math.abs(rightBack.getCurrentPosition() - rightBack.getTargetPosition());
            return sum / 4;
        }

        public boolean isBusy()
        {
            int cnt = 0;
            if(leftFront.isBusy())  cnt++;
            if(leftBack.isBusy())   cnt++;
            if(rightFront.isBusy()) cnt++;
            if(rightBack.isBusy())  cnt++;
            return (cnt > 1);
        }

        public void showPositions()
        {
            telemetry.addData("TicksPerRev", ticksPerRevolution);

            telemetry.addData("lf current", leftFront.getCurrentPosition());
            telemetry.addData("rf current", rightFront.getCurrentPosition());
            telemetry.addData("lb current", leftBack.getCurrentPosition());
            telemetry.addData("rb current", rightBack.getCurrentPosition());

            telemetry.addData("lf target", leftFront.getTargetPosition());
            telemetry.addData("rf target", rightFront.getTargetPosition());
            telemetry.addData("lb target", leftBack.getTargetPosition());
            telemetry.addData("rb target", rightBack.getTargetPosition());

            telemetry.update();
        }
    }

    public class Collector
    {
        public DcMotor rotLeft, rotRight, extender;
        public CRServo mat;
        public Servo box;
        public int posBox;

        public double initPos = 0.25;
        public double upPos = 0.15;
        public double dropPos = 0.35;


        Collector(DcMotor _rotLeft, DcMotor _rotRight, DcMotor _extend, CRServo _maturique, Servo _box)
        {
            rotLeft = _rotLeft; rotRight = _rotRight; extender = _extend; mat = _maturique; box = _box;
            mat.setDirection(DcMotorSimple.Direction.FORWARD);
            box.setPosition(initPos);
            posBox = 1;
            rotLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            rotRight.setDirection(DcMotorSimple.Direction.REVERSE);
            extender.setDirection(DcMotorSimple.Direction.FORWARD);
            rotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void afterInitStart()
        {
            rotLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rotRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void rotate(double speed)
        {
            rotLeft.setPower(speed);
            rotRight.setPower(speed);

            telemetry.addData("Zero left", rotLeft.getZeroPowerBehavior().toString());
            telemetry.addData("Zero Right", rotRight.getZeroPowerBehavior().toString());
        }

        public void extend(double speed)
        {
            extender.setPower(speed);
        }

        public void collect(int direction)
        {
            if(direction == 0)
            {
                mat.setPower(0.0);
                mat.getController().pwmDisable();
            }
            else if(direction == 1)
            {
                mat.getController().pwmEnable();
                mat.setPower(1.0);
            }
            else
            {
                mat.getController().pwmEnable();
                mat.setPower(-1.0);
            }
        }

        public void boxUp()
        {
            posBox++;
            if(posBox > 2)  posBox = 2;
            setBoxPosition(posBox);
        }
        public void boxDown()
        {
            posBox--;
            if(posBox < 0)  posBox = 0;
            setBoxPosition(posBox);
        }

        public void setBoxPosition(int pos)
        {
            if(pos == 0)    box.setPosition(initPos);
            if(pos == 1)    box.setPosition(upPos);
            if(pos == 2)    box.setPosition(dropPos);
        }
    }

    public class Lifter
    {
        public DcMotor motor;
        public final static int tickInterval = 5500;

        Lifter(DcMotor _motor)
        {
            motor = _motor;
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void afterStartInit()
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void move(double speed)
        {
            motor.setPower(speed);
        }

        public void goToPosition(int ticks, double power)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(ticks);

            while(motor.isBusy())
            {
                if(!opmode.opModeIsActive())
                {
                    motor.setPower(0);
                    return;
                }

                motor.setPower(power);
            }
            motor.setPower(0.0);

            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void land()
        {
            goToPosition(tickInterval, 1.0);
        }
    }

    public class MineralIdentifier
    {
        public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
        public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
        public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
        public static final String VUFORIA_KEY = Config.vuforiaKey;
        public VuforiaLocalizer vuforia;
        public TFObjectDetector tfod;
        public double cameraX = 1280;
        public double middleX = cameraX / 2.0;
        public int last = -1;
        public IdentifierType type = IdentifierType.ALL;

        MineralIdentifier() { ; }

        public void setType(IdentifierType _type) { type = _type; }

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
        
        public void init()
        {
            initVuforia();
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }
            telemetry.addData("Vuforia intialized!", " ");
            start();
        }

        public void start() { tfod.activate(); }
        public void stop() { tfod.shutdown(); }

        public boolean valid(Recognition r)
        {
            double ypos = ( (r.getTop() + r.getBottom()) / 2.0 );
            if(ypos < 250.0)    return false;
            return true;
        }

        public List<Recognition> validRecognitions(List<Recognition> rec)
        {
            ArrayList<Recognition> list = new ArrayList<Recognition>();
            for(Recognition r: rec)
                if(valid(r))
                    list.add(r);
            return list;
        }

        public double getPosition(Recognition rec)
        {
            return ( (rec.getLeft() + rec.getRight()) / 2.0 );
        }

        public int getGoldMineral()
        {
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

            if(type == IdentifierType.ALL)
            {
                int allcnt = recognitions.size(), goldcnt = 0;
                for(Recognition rec: recognitions)
                    if(rec.getLabel().equals(LABEL_GOLD_MINERAL))   goldcnt++;

                if(allcnt != 3 && goldcnt != 1) return -1;

                double goldx = 0, silverx1 = 0, silverx2 = 0;
                for(Recognition rec: recognitions)
                {
                    if(rec.getLabel().equals(LABEL_GOLD_MINERAL))
                        goldx = getPosition(rec);
                    else if(rec.getLabel().equals(LABEL_SILVER_MINERAL))
                    {
                        if(silverx1 == 0)   silverx1 = getPosition(rec);
                        else    silverx2 = getPosition(rec);
                    }
                }

                if(silverx1 > silverx2) { double aux = silverx1; silverx1 = silverx2; silverx2 = aux; }

                if(goldx < silverx1)    return 0;
                if(goldx < silverx2)    return 1;
                return 2;
            }
            else if(type == IdentifierType.LEFT_MID)
            {
                int goldcnt = 0;
                for(Recognition rec: recognitions)
                    if(rec.getLabel().equals(LABEL_GOLD_MINERAL))   goldcnt++;
                if(goldcnt == 0)    return 2;
                if(goldcnt > 1) return -1;
                for(Recognition rec: recognitions)
                    if(rec.getLabel().equals(LABEL_GOLD_MINERAL))
                    {
                        double pos = getPosition(rec);
                        if(pos > middleX)   return 1;
                        else    return 0;
                    }
                return -1;
            }
            else if(type == IdentifierType.MID_RIGHT)
            {
                int goldcnt = 0;
                for(Recognition rec: recognitions)
                    if(rec.getLabel().equals(LABEL_GOLD_MINERAL))   goldcnt++;
                if(goldcnt == 0)    return 0;
                if(goldcnt > 1) return -1;
                for(Recognition rec: recognitions)
                    if(rec.getLabel().equals(LABEL_GOLD_MINERAL))
                    {
                        double pos = getPosition(rec);
                        if(pos > middleX)   return 2;
                        else    return 1;
                    }
                return -1;
            }
            return -1;
        }

        public int findGold()
        {
            //telemetry.setAutoClear(false);
            //start();
            //opmode.sleep(1000);
            int count = 10;
            int[] fq = new int[3];
            for(int i = 0; i < count; i++)
            {
                int where = getGoldMineral();
                telemetry.addData(new Integer(i).toString(), where);
                telemetry.update();
                if(where != -1)
                    fq[where]++;
                last = where;
            }
            telemetry.addData("fq 0", fq[0]);
            telemetry.addData("fq 1", fq[1]);
            telemetry.addData("fq 2", fq[2]);
            telemetry.update();
            int mx = -1, id = -1;
            for(int i = 0; i < 3; i++)
                if(fq[i] > mx)
                {
                    mx = fq[i];
                    id = i;
                }
            stop();
            return id;
        }
    }

    public class Autonomous
    {
        public BNO055IMU imu;
        public ModernRoboticsI2cRangeSensor front;
        public double myAngle;

        Autonomous() { ; }

        public void init()
        {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.mode = BNO055IMU.SensorMode.IMU;
            //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            //parameters.loggingEnabled      = true;
            //parameters.loggingTag          = "IMU";
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            myAngle = getHeading();
            front = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, Config.frontSensor);

            while(!imu.isGyroCalibrated())
            {
                telemetry.addData("Gyro", "Calibrating...");
                telemetry.update();
            }
            telemetry.addData("Gyro", "Calibrated");
            telemetry.update();
        }

        public double getFrontDistance() { return front.getDistance(DistanceUnit.MM); }

        public void land() { lift.land(); }

        public void startTracking() { imu.startAccelerationIntegration(new Position(), new Velocity(), 200); }
        public void stopTracking() { imu.stopAccelerationIntegration(); }
        public void showPosition()
        {
            Position p = imu.getPosition();
            telemetry.addData("Pos x", p.x);
            telemetry.addData("Pos y", p.y);
            telemetry.addData("Pos z", p.z);
            telemetry.addData("Unit", p.unit.toString());
        }

        public double degToRad(double x) { return (x * Math.PI) / 180.0; }
        public double radToDeg(double x) { return (x * 180.0) / Math.PI; }

        public Orientation getGyro() { return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); }

        public double getHeading() { return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; }
        public double getHeadingRadians() { return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle; }

        public double getAngleDistance(double start, double fin)
        {
            start = AngleUnit.normalizeDegrees(start);
            fin = AngleUnit.normalizeDegrees(fin);
            double dist = fin - start;
            dist = AngleUnit.normalizeDegrees(dist);
            return dist;
        }

        public double getPower(double pw)
        {
            return pw;
        }

        public void rotateTo(double degrees)
        {
            degrees = AngleUnit.normalizeDegrees(degrees);
            double dist = getAngleDistance(getHeading(), degrees);
            rotateP(dist);
        }

        public void rotateP(double degrees)
        {
            runner.reset(DcMotor.RunMode.RUN_USING_ENCODER);
            double accepted = 0.5;
            double needAngle = AngleUnit.normalizeDegrees(getHeading() + degrees);
            double lastPower = 0.0;
            double maxDifference = 0.1;
            double angleDecrease = 45.0;
            while( true )
            {
                double myAngle = getHeading();
                telemetry.addData("Heading", myAngle);
                telemetry.addData("Power", lastPower);
                double distance = getAngleDistance(myAngle, needAngle);
                telemetry.addData("Distance", distance);
                telemetry.update();
                if(Math.abs(distance) < accepted) break;

                double power = 0.0;

                if(Math.abs(distance) < angleDecrease)
                    power = Math.abs(distance) / angleDecrease;
                else
                    power = 1.0;

                power = Math.min(power, lastPower + maxDifference);
                power = Math.max(power, lastPower - maxDifference);
                lastPower = power;
                power = - power;
                if(distance < 0)    power = -power;
                power = getPower(power);

                runner.angleMove(0, 0, power);

                if(!opmode.opModeIsActive())
                {
                    runner.stop();
                    return;
                }
            }
            runner.stop();
        }

        public int distanceToTicks(double dist)
        {
            double ans = (dist * ticksPerRevolution) / wheelCircumference;
            return (int)ans;
        }

        public void moveStraight(double distance, double angle)
        {
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
            int ticksDecrease = (int)(2.5 * ticksPerRevolution);
            double angleDecrease = 45.0;

            double heading = getHeading();

            while( runner.isBusy() )
            {
                double power = 0.0;

                int ticksDistance = runner.getTicksDistance();

                if(ticksDistance < ticksDecrease)
                    power = (double)ticksDistance / (double)ticksDecrease;
                else
                    power = 1.0;

                power = Math.min(power, lastPower + maxDifference);
                power = Math.max(power, lastPower - maxDifference);
                lastPower = power;
                power = Math.max(power, 0.05);

                double angleDistance = getAngleDistance(getHeading(), heading);
                double rot = 0.0;
                if(Math.abs(angleDistance) > angleDecrease)
                {
                    if(angleDistance < 0)   rot = 1.0;
                    else    rot = -1.0;
                }
                else
                    rot = -(angleDistance / angleDecrease);

                Runner.MotorPowers pw = runner.angleDrive(power, angle, rot);
                runner.setPower(pw);

                runner.showPositions();

                if(!opmode.opModeIsActive())
                {
                    runner.stop();
                    return;
                }
            }
            runner.stop();
        }

        public void move(double distance, double angle)
        {
            //moveStraight(distance, angle);
            //if(distance != -23232)  return;

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
            int ticksDecrease = (int)(1.25 * ticksPerRevolution);

            while( runner.isBusy() )
            {
                double power = 0.0;

                int ticksDistance = runner.getTicksDistance();

                if(ticksDistance < ticksDecrease)
                    power = (double)ticksDistance / (double)ticksDecrease;
                else
                    power = 1.0;

                power = Math.min(power, lastPower + maxDifference);
                power = Math.max(power, lastPower - maxDifference);
                lastPower = power;
                power = Math.max(power, 0.05);

                Runner.MotorPowers pw = runner.angleDrive(power, angle, 0);
                runner.setPower(pw);

                runner.showPositions();

                if(!opmode.opModeIsActive())
                {
                    runner.stop();
                    return;
                }
            }
            runner.stop();
        }

        public void moveUntilDistance(double distance, double angle)
        {
            angle += Math.PI / 2.0;
            runner.reset(DcMotor.RunMode.RUN_USING_ENCODER);
            double accepted = 5.0;
            double lastPower = 0.0;
            double maxDifference = 0.1;
            double distanceDecrease = 500.0;
            double maxPower = 0.85;
            while( true )
            {
                double myDistance = getFrontDistance();
                telemetry.addData("Distance", myDistance);
                telemetry.addData("Power", lastPower);
                double rem = myDistance - distance;
                telemetry.addData("Remaining", rem);
                telemetry.update();
                if(rem < accepted) break;

                double power = 0.0;
                if(rem < distanceDecrease)
                    power = rem / distanceDecrease;
                else
                    power = 1.0;

                power = Math.min(power, lastPower + maxDifference);
                power = Math.max(power, lastPower - maxDifference);
                lastPower = power;

                runner.angleMove(power * maxPower, angle, 0);

                if(!opmode.opModeIsActive())
                {
                    runner.stop();
                    return;
                }
            }
            runner.stop();
        }
    }

    public Runner runner;
    public Collector collector;
    public MineralIdentifier identifier;
    public Autonomous autonomous;
    public Lifter lift;

    Mugurel(HardwareMap hm)
    {
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
                hm.get(CRServo.class, Config.maturique),
                hm.get(Servo.class, Config.box)
        );
        lift = new Lifter( hm.get(DcMotor.class, Config.lift) );
        identifier = new MineralIdentifier();
        autonomous = new Autonomous();
    }

    public void initTelemetry(Telemetry _t) { telemetry = _t; }
    public void setOpmode(LinearOpMode _opmode) { opmode = _opmode; }

    public void afterStartInit()
    {
        runner.afterStartInit();
        collector.afterInitStart();
        lift.afterStartInit();
    }
}
