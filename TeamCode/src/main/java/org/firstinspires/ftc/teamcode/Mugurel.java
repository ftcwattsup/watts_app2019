package org.firstinspires.ftc.teamcode;

import android.text.style.LineHeightSpan;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import javax.lang.model.type.NullType;

public class Mugurel
{
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

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
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void afterStartInit()
        {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void setMode(DcMotor.RunMode mode) { leftFront.setMode(mode); leftBack.setMode(mode); rightFront.setMode(mode); rightBack.setMode(mode); }

        public void setPower(MotorPowers pw) { setPower(pw.lf, pw.lb, pw.rf, pw.rb); }
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
        public MotorPowers basicDrive(double x, double y, double r)
        {
            double pw[] = new double[4];
            pw[0] = x + y + r;
            pw[1] = -x + y - r;
            pw[2] = -x + y + r;
            pw[3] = x + y - r;

            MotorPowers mpw = new MotorPowers(pw[0], pw[2], pw[1], pw[3]);
            mpw.normalize();
            return mpw;
        }

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
    }

    public class Collector
    {
        DcMotor rotLeft, rotRight, extender;
        CRServo mat;

        Collector(DcMotor _rotLeft, DcMotor _rotRight, DcMotor _extend, CRServo _maturique)
        {
            rotLeft = _rotLeft; rotRight = _rotRight; extender = _extend; mat = _maturique;
            mat.setDirection(DcMotorSimple.Direction.FORWARD);
            rotLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            rotRight.setDirection(DcMotorSimple.Direction.REVERSE);
            extender.setDirection(DcMotorSimple.Direction.FORWARD);
            rotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
    }

    public class Lifter
    {
        public DcMotor motor;

        Lifter(DcMotor _motor)
        {
            motor = _motor;
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
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

        MineralIdentifier() { ; }

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
        }

        public void start() { tfod.activate(); }
        public void stop() { tfod.shutdown(); }

        public int getGoldMineral()
        {
            /**
             * -1 = invalid
             * 0 = left
             * 1 = middle
             * 2 = right
             */
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if(updatedRecognitions == null) return -1;
            int goldCnt = 0;
            for(Recognition rec: updatedRecognitions)
                if(rec.getLabel().equals(LABEL_GOLD_MINERAL))
                    goldCnt++;

            if(goldCnt > 1)    return -1;
            if(goldCnt == 0)    return 2;

            for(Recognition rec: updatedRecognitions)
                if(rec.getLabel().equals(LABEL_GOLD_MINERAL))
                {
                    double pos = (rec.getLeft() + rec.getRight()) / 2.0;
                    telemetry.addData("Position", pos);
                    if(pos > middleX)   return 1;
                    else return 0;
                }
            return 0;
        }

        public int findGold()
        {
            int count = 10;
            int[] fq = new int[3];
            for(int i = 0; i < count; i++)
            {
                int where = getGoldMineral();
                telemetry.addData(new Integer(i).toString(), where);
                if(where != -1)
                    fq[where]++;
            }
            telemetry.addData("fq 0", fq[0]);
            telemetry.addData("fq 1", fq[1]);
            telemetry.addData("fq 2", fq[2]);
            int mx = -1, id = -1;
            for(int i = 0; i < 3; i++)
                if(fq[i] > mx)
                {
                    mx = fq[i];
                    id = i;
                }
            return id;
        }
    }

    public class Autonomous
    {
        public BNO055IMU imu;
        public DistanceSensor distanceSensor;
        public double goodAngle;

        Autonomous()
        {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            goodAngle = 0;
            distanceSensor = hardwareMap.get(DistanceSensor.class, Config.downSensor);
        }

        public void setGoodAngle() { goodAngle = getHeading(); }

        public double getHeading()
        {
            return imu.getAngularOrientation().firstAngle;
        }

        public void rotateToAngle(double angle)
        {

        }

        public void rotate(double angle)
        {

        }

        public void moveDistance(double angle, double distance)
        {
            ;
        }

        public double getDownDistance()
        {
            return distanceSensor.getDistance(DistanceUnit.MM);
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
                hm.get(CRServo.class, Config.maturique)
        );
        lift = new Lifter( hm.get(DcMotor.class, Config.lift) );
        //identifier = new MineralIndetifier();
        //autonomous = new Autonomous();
    }

    public void initTelemetry(Telemetry _t) { telemetry = _t; }

    public void afterStartInit()
    {
        runner.afterStartInit();
        collector.afterInitStart();
        lift.afterStartInit();
    }
}
