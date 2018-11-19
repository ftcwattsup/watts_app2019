package org.firstinspires.ftc.teamcode;

import android.text.style.LineHeightSpan;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mugurel
{
    public enum Face { FRONT, BACK };
    public enum CollectionType { COLLECT, SPIT, STOP };
    public enum ExtenderDirection { EXTEND, CONTRACT, STOP };

    public class Runner
    {
        Face face;
        private DcMotor leftFront, rightFront, leftBack, rightBack;
        private DcMotor initialLeftFront, initialRightFront, initialLeftBack, initialRightBack;

        private static final double eps = 1e-6;

        Runner(DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb)
        {
            initialLeftFront = lf; initialRightFront = rf; initialLeftBack = lb; initialRightBack = rb;
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            facing(Face.FRONT);
        }

        public void afterStartInit()
        {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void setMode(DcMotor.RunMode mode) { leftFront.setMode(mode); leftBack.setMode(mode); rightFront.setMode(mode); rightBack.setMode(mode); }

        public void setPower(double left, double right, double rat)
        {
            leftFront.setPower(left * rat);
            leftBack.setPower(left * rat);
            rightFront.setPower(right * rat);
            rightBack.setPower(right * rat);
        }
        public void setPower(double left, double right) { setPower(left, right, 1.0); }

        public void facing(Face f)
        {
            if(face == f)   return;

            if(f == Face.FRONT)
            {
                leftFront = initialLeftFront; leftBack = initialLeftBack; rightFront = initialRightFront; rightBack = initialRightBack;
            }

            if(f == Face.BACK)
            {
                leftFront = initialRightBack; leftBack = initialRightFront; rightFront = initialLeftBack; rightBack = initialLeftFront;
            }

            leftFront.setDirection(DcMotorSimple.Direction.FORWARD); leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE); rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public double scale(double x)
        {
            return x;
            /// TODO: try x^3
        }

        public void move(double x, double y, double r, double rat)
        {
            x = scale(x); y = scale(y); r = scale(r);

            if( Math.abs(r) < eps ) setPower(y, y, rat);
            else if( Math.abs(y) < eps )    setPower(-r, r, rat);
            else if( r < 0.0 )  setPower(y * (1.0 + r), y, rat);
            else if( r > 0.0 )  setPower(y, y * (1.0 - r), rat);
        }
        public void move(double x, double y, double r) { move(x, y, r, 1.0); }
    }

    public class Collector
    {
        private DcMotor rotor, extender, collector;

        private final double collectorPower = 0.75;
        private final double extenderPower = 0.75;
        private CollectionType collectorState;

        Collector(DcMotor r, DcMotor e, DcMotor c)
        {
            rotor = r; extender = e; collector = c;

            rotor.setDirection(DcMotorSimple.Direction.FORWARD);
            extender.setDirection(DcMotorSimple.Direction.FORWARD);
            collector.setDirection(DcMotorSimple.Direction.FORWARD);

            rotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            collector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void afterStartInit()
        {
            rotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void setRotationPower(double power) { rotor.setPower(power); }

        public void extend(ExtenderDirection direction)
        {
            if(direction == ExtenderDirection.EXTEND)   extender.setPower(extenderPower);
            else if(direction == ExtenderDirection.CONTRACT)    extender.setPower(-extenderPower);
            else if(direction == ExtenderDirection.STOP)    extender.setPower(0.0);
        }

        public void collect(CollectionType type)
        {
            if(type == CollectionType.COLLECT)  collector.setPower(collectorPower);
            else if(type == CollectionType.SPIT)    collector.setPower(-collectorPower);
            else if(type == CollectionType.STOP)    collector.setPower(0.0);
            collectorState = type;
        }

        public void collectorPress(CollectionType type)
        {
            if(type == collectorState)  collect(CollectionType.STOP);
            else    collect(type);
        }
    }

    public Runner runner;
    public Collector collector;

    Mugurel(HardwareMap hm)
    {
        runner = new Runner(
                hm.get(DcMotor.class, Config.leftFront),
                hm.get(DcMotor.class, Config.rightFront),
                hm.get(DcMotor.class, Config.leftBack),
                hm.get(DcMotor.class, Config.rightBack)
        );

        collector = new Collector(
                hm.get(DcMotor.class, Config.rotor),
                hm.get(DcMotor.class, Config.extender),
                hm.get(DcMotor.class, Config.collector)
        );
    }

    public void afterStartInit()
    {
        runner.afterStartInit();
        collector.afterStartInit();
    }
}
