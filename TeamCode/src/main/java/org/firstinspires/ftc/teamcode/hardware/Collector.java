package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedList;
import java.util.Queue;

public class Collector {

    public Telemetry telemetry;
    public LinearOpMode opmode;
    public ElapsedTime timer;

    class RotationOperation {
        public int ticks = 0;
        public double power = 0;
        public double powerFinal = 0;

        public RotationOperation() { ; }
        public RotationOperation(int _ticks, double _power) { ticks = _ticks; power = _power; powerFinal = _power; }
        public RotationOperation(int _ticks, double _power, double _powerf) { ticks = _ticks; power = _power; powerFinal = _powerf; }
    }

    Queue<RotationOperation> queue;

    public DcMotor rot, extender, mat;
    public Servo holder;
    public int rotTicks = 70;
    public double defaultPower = 0.9;

    public int extendLander = 4900;
    public int extendMax = 5450;

    public double matRevolution = 100;

    public double holdClosed = 0.3;
    public double holdOpen = 0.08;

    public RotationOperation nowPlaying = new RotationOperation();

    Collector(DcMotor _rot, DcMotor _extend, DcMotor _maturique, Servo _holder) {
        rot = _rot;
        extender = _extend;
        mat = _maturique;
        holder = _holder;

        holder.setPosition(holdClosed);

        mat.setDirection(DcMotorSimple.Direction.REVERSE);
        rot.setDirection(DcMotorSimple.Direction.FORWARD);
        extender.setDirection(DcMotorSimple.Direction.FORWARD);

        rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        matRevolution = mat.getMotorType().getTicksPerRev();

        queue = new LinkedList<RotationOperation>();
    }

    public void afterInitStart() {
        double power = defaultPower;
        rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rot.setTargetPosition(0);
        rot.setPower(power);

        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Holder
     */

    public void openHolder() { holder.setPosition(holdOpen); }
    public void closeHolder() { holder.setPosition(holdClosed); }

    /**
     * Rotation
     */

    public void update()
    {
        if(rot.isBusy()) {
            int ticksDecrease = 800;
            int distance = Math.abs(rot.getCurrentPosition() - rot.getTargetPosition());
            if(distance > ticksDecrease)    return;
            double power = nowPlaying.powerFinal + ((double)(distance) / (double)(ticksDecrease)) * (nowPlaying.power - nowPlaying.powerFinal);
            rot.setPower(power);
            return;
        }
        if(queue.isEmpty()) return;
        RotationOperation op = queue.peek();
        queue.remove();
        addTicksWithPower(op);
    }

    public void addTicksGamepad(double y) {
        if(Math.abs(y) < 0.001) return;
        stopRotation();
        addTicks((int)(y * rotTicks));
    }

    public void addTicks(int ticks) {
        addTicksWithPower(ticks, defaultPower);
    }

    public void addTicksWithPower(int ticks, double power) {
        //rot.setPower(power);
        //rot.setTargetPosition(rot.getCurrentPosition() + ticks);

        queue.add(new RotationOperation(ticks, power));
    }

    public void addTicksWithPower(int ticks, double power, double powerf) {
        //rot.setPower(power);
        //rot.setTargetPosition(rot.getCurrentPosition() + ticks);

        queue.add(new RotationOperation(ticks, power, powerf));
    }

    public void rotateTicks(int ticks) {
        rot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rot.setTargetPosition(ticks);
        rot.setPower(1.0);
        //while(rotLeft.isBusy() && rot.isBusy()) {
        //    if(!opmode.opModeIsActive())    return;
        //}
    }

    public void addTicksWithPower(RotationOperation op) {
        nowPlaying = op;
        int ticks = op.ticks;
        double power = op.power;

        rot.setPower(power);
        rot.setTargetPosition(rot.getCurrentPosition() + ticks);
    }

    public void stopRotation() {
        rot.setTargetPosition(rot.getCurrentPosition());
        extender.setPower(0.0);
        while(!queue.isEmpty()) queue.remove();
    }

    public void noEncoder(double y) {
        if(Math.abs(y) < 0.001) {
            rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rot.setTargetPosition(rot.getCurrentPosition());
        }
        else {
            rot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rot.setPower(y);
        }
    }

    public void rotateToPosition(int pos, double power) {
        rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rot.setTargetPosition(pos);
        rot.setPower(power);
    }

    public void rotateToPositionWait(int pos, double power) {
        rot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rot.setTargetPosition(pos);
        rot.setPower(power);

        while(rot.isBusy())
        {
            if(!opmode.opModeIsActive())
            {
                rot.setPower(0);
                return;
            }
        }
    }

    /**
     * Extension
     */
    public void extend(double speed) {
        if(extender.isBusy() && Math.abs(speed) < 0.01)   return;
        extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extender.setPower(speed);
    }

    public void goToLanderPosition() {
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setTargetPosition(extendLander);
        extender.setPower(1.0);
    }

    public void extendGoToPosition(int pos, double power) {
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setTargetPosition(pos);
        extender.setPower(power);
    }

    public void extendGoToPositionWait(int pos, double power) {
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setTargetPosition(pos);
        extender.setPower(power);
        while(extender.isBusy())
        {
            if(!opmode.opModeIsActive())
            {
                extender.setPower(0);
                return;
            }
        }
    }

    public void extendGoToPosition(int pos) {
        extendGoToPosition(pos, 1.0);
    }

    /**
     * Maturique
     */
    public void collect(double power) {
        if(Math.abs(power) < 0.01)  stopMaturique();
        else    startMaturique(power);
    }

    public void startMaturique(double power) {
        if(mat.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)    mat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mat.setPower(power);
    }

    public void stopMaturique() {
        mat.setPower(0);
        return;
        /*int ticks = mat.getCurrentPosition();
        mat.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int period = (int)(matRevolution) / 4;
        int newpos = (int)(ticks / period) * period;

        int pos = newpos;
        int dst = Math.abs(mat.getCurrentPosition() - newpos);

        newpos -= period;
        if(Math.abs(mat.getCurrentPosition() - newpos) < dst) {
            dst = Math.abs(mat.getCurrentPosition() - newpos);
            pos = newpos;
        }

        newpos += 2 * period;
        if(Math.abs(mat.getCurrentPosition() - newpos) < dst) {
            dst = Math.abs(mat.getCurrentPosition() - newpos);
            pos = newpos;
        }

        mat.setTargetPosition(pos);
        mat.setPower(1.0);*/
    }

    public void rotateMat()
    {
        mat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mat.setPower(0);
        mat.setTargetPosition((int)matRevolution);
        mat.setPower(1.0);
    }

    public void showTelemetry() {
        telemetry.addData("rot current", rot.getCurrentPosition());
        telemetry.addData("rot target", rot.getTargetPosition());

        telemetry.addData("mat current", mat.getCurrentPosition());
        telemetry.addData("mat target", mat.getTargetPosition());
        telemetry.addData("mat power", mat.getPower());
        telemetry.addData("mat revolution", matRevolution);

        telemetry.addData("ext current", extender.getCurrentPosition());
        telemetry.addData("ext target", extender.getTargetPosition());
        telemetry.addData("ext power", extender.getPower());

        //telemetry.update();
    }

    public void setTelemetry(Telemetry _t) { telemetry = _t; }
    public void setOpmode(LinearOpMode _o) { opmode = _o; }
}
