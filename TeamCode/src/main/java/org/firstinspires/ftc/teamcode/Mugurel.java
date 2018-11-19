package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mugurel
{
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor rotor, extender, collector;

    Mugurel(HardwareMap hm)
    {
        frontLeft = hm.get(DcMotor.class, Config.frontLeft);
        frontRight = hm.get(DcMotor.class, Config.frontRight);
        backLeft = hm.get(DcMotor.class, Config.backLeft);
        backRight = hm.get(DcMotor.class, Config.backRight);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        rotor = hm.get(DcMotor.class, Config.rotor);
        extender = hm.get(DcMotor.class, Config.extender);
        collector = hm.get(DcMotor.class, Config.collector);
    }

    public void setPower(double left, double right, double rat)
    {
        frontLeft.setPower(left * rat);
        backLeft.setPower(left * rat);
        frontRight.setPower(right * rat);
        backRight.setPower(right * rat);
    }

    public void move(double x, double y, double r, double rat)
    {
        if(r == 0)
            setPower(y, y, rat);
        else if(y == 0)
            setPower(-r, r, rat);
        else if(x < 0)
            setPower(-y - y * x, y, rat);
        else if(x > 0)
            setPower(-y, y - y * x, rat);
    }

    public void move(double x, double y, double r) { move(x, y, r, 1.0); }
}
