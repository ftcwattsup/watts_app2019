package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Runner {
    public Telemetry telemetry;
    public LinearOpMode opmode;

    public DcMotor leftFront, rightFront, leftBack, rightBack;
    public double faceAngle;
    public final double wheelAngle = Math.PI / 4.0;

    /**
     * Motor functions
     */

    public class MotorPowers {
        public double lf, lb, rf, rb;

        MotorPowers() {
            lf = lb = rf = rb = 0;
        }

        MotorPowers(double _lf, double _lb, double _rf, double _rb) {
            lf = _lf;
            lb = _lb;
            rf = _rf;
            rb = _rb;
        }

        public void normalize() {
            double mx = Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.max(Math.abs(rf), Math.abs(rb)));
            if (mx > 1.0) {
                lf /= mx;
                lb /= mx;
                rf /= mx;
                rb /= mx;
            }
        }

        public void speed(double spd) {
            spd = Math.abs(spd);

            if (spd > 1.0) spd = 1.0;
            double mx = Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.max(Math.abs(rf), Math.abs(rb)));
            if (spd <= 1.0 && mx < spd) {
                double coef = spd / mx;
                lf *= coef;
                lb *= coef;
                rf *= coef;
                rb *= coef;
            }
        }

        public void rap(double r) {
            lf *= r;
            lb *= r;
            rf *= r;
            rb *= r;
        }
    }

    public Runner(DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb) {
        leftFront = lf;
        rightFront = rf;
        leftBack = lb;
        rightBack = rb;
        faceAngle = 0;
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void afterStartInit() {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    public void setPower(MotorPowers pw) {
        setPower(pw.lf, pw.lb, pw.rf, pw.rb);
    }

    public void setPower(double pw) {
        setPower(pw, pw, pw, pw);
    }

    public void setPower(double lf, double lb, double rf, double rb) {
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
    public MotorPowers angleDriveFromAxes(double x, double y, double r) {
        double angle = Math.atan2(y, x);
        telemetry.addData("Angle", angle);
        while (angle < 0) angle += 2 * Math.PI;
        double speed = Math.sqrt(x * x + y * y);
        telemetry.addData("Speed", speed);
        return angleDrive(speed, angle, r);
    }

    public MotorPowers angleDrive(double speed, double angle, double rot) {
        angle += faceAngle;
        angle += wheelAngle;

        double lf = speed * Math.sin(-angle) - rot;
        double rf = speed * Math.cos(-angle) + rot;
        double lb = speed * Math.cos(-angle) - rot;
        double rb = speed * Math.sin(-angle) + rot;
        lf = -lf;
        rf = -rf;
        lb = -lb;
        rb = -rb;

        MotorPowers mpw = new MotorPowers(lf, lb, rf, rb);

        mpw.normalize();
        mpw.speed(speed);
        return mpw;
    }

    public void move(double x, double y, double r) {
        move(x, y, r, 1.0);
    }

    public void move(double x, double y, double r, double rap) {
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

    public void angleMove(double speed, double angle, double rot) {
        angleMove(speed, angle, rot, 1.0);
    }

    public void angleMove(double speed, double angle, double rot, double rap) {
        MotorPowers pw = angleDrive(speed, angle, rot);
        pw.rap(rap);
        setPower(pw);
    }

    public void moveTank(double x, double y, double r) {
        double left = y + r, right = y - r;
        if (Math.abs(r) < 0.001) setPower(left, left, right, right);
        else if (Math.abs(y) < 0.001) setPower(left, left, right, right);
        else {
            if (left < -1.0) left = -1.0;
            if (left > 1.0) left = 1.0;
            if (right < -1.0) right = -1.0;
            if (right > 1.0) right = 1.0;
            setPower(left, left, right, right);
        }
    }

    public void setFace(double angle) {
        faceAngle = angle;
    }
    public void setFaceDegrees(double angle) { setFace(Math.toRadians(angle)); }

    /**
     * To position (autonomous)
     */
    public void reset(DcMotor.RunMode mode) {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(mode);
    }

    public void reset() {
        reset(leftFront.getMode());
    }

    public void stop() {
        setPower(0);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setTargetPositions(int lf, int lb, int rf, int rb) {
        leftFront.setTargetPosition(lf);
        leftBack.setTargetPosition(lb);
        rightFront.setTargetPosition(rf);
        rightBack.setTargetPosition(rb);
    }

    public int getTicksDistance() {
        int sum = 0;
        sum += Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition());
        sum += Math.abs(leftBack.getCurrentPosition() - leftBack.getTargetPosition());
        sum += Math.abs(rightFront.getCurrentPosition() - rightFront.getTargetPosition());
        sum += Math.abs(rightBack.getCurrentPosition() - rightBack.getTargetPosition());
        return sum / 4;
    }

    public boolean isBusy() {

        final int LIMIT = 10;
        int rem = getTicksDistance();
        if(rem <= LIMIT)    return false;
        return true;

        /*int cnt = 0;
        if (leftFront.isBusy()) cnt++;
        if (leftBack.isBusy()) cnt++;
        if (rightFront.isBusy()) cnt++;
        if (rightBack.isBusy()) cnt++;
        return (cnt > 1);*/
    }

    public void showPositions() {
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

    public void setTelemetry(Telemetry _t) { telemetry = _t; }
    public void setOpmode(LinearOpMode _o) { opmode = _o; }
}
