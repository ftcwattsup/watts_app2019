package org.firstinspires.ftc.teamcode;

import android.text.style.LineHeightSpan;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import javax.lang.model.type.NullType;

public class Mugurel
{
    public Telemetry telemetry;

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

            telemetry.addData("Face", faceAngle);

            telemetry.addData("Dir lf", leftFront.getDirection().toString());
            telemetry.addData("Dir rf", rightFront.getDirection().toString());
            telemetry.addData("Dir lb", leftBack.getDirection().toString());
            telemetry.addData("Dir rb", rightBack.getDirection().toString());

            telemetry.addData("Front Left", leftFront.getPower());
            telemetry.addData("Front Right", rightFront.getPower());
            telemetry.addData("Back Left", leftBack.getPower());
            telemetry.addData("Back Right", rightBack. getPower());
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

    public Runner runner;

    Mugurel(HardwareMap hm)
    {
        runner = new Runner(
                hm.get(DcMotor.class, Config.leftFront),
                hm.get(DcMotor.class, Config.rightFront),
                hm.get(DcMotor.class, Config.leftBack),
                hm.get(DcMotor.class, Config.rightBack)
        );
    }

    public void initTelemetry(Telemetry _t) { telemetry = _t; }

    public void afterStartInit()
    {
        runner.afterStartInit();
    }
}
