package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.sun.source.tree.Tree;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

public class MyGamepad
{
    public class Toggle
    {
        boolean value = false, pressed = false;
        Toggle() { ; }

        public boolean query(boolean press)
        {
            if(press)
            {
                if(!pressed)
                {
                    value ^= true;
                    pressed = true;
                    return true;
                }
            }
            else
                pressed = false;
            return false;
        }
    }

    public Gamepad gamepad;
    public TreeMap<Buttons, Toggle> toggleMap;
    public TreeMap<Buttons, PressType> typeMap;
    public TreeMap<Buttons, Boolean> valueMap;

    MyGamepad(Gamepad gp)
    {
        gamepad = gp;
        toggleMap = new TreeMap();
        typeMap = new TreeMap();
        valueMap = new TreeMap();
        for(Buttons btn: Buttons.values())
        {
            toggleMap.put(btn, new Toggle());
            typeMap.put(btn, PressType.PRESS);
            valueMap.put(btn, false);
        }
    }

    public enum Buttons
    {
        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT,
        A,
        B,
        X,
        Y,
        START,
        BACK,
        LEFT_BUMPER,
        RIGHT_BUMPER,
        LEFT_JOYSTICK,
        RIGHT_JOYSTICK
    }

    public enum PressType
    {
        PRESS,
        TOGGLE
    }

    public enum Axes
    {
        LEFT_X,
        LEFT_Y,
        RIGHT_X,
        RIGHT_Y,
        LEFT_TRIGGER,
        RIGHT_TRIGGER
    }

    public boolean getRawValue(Buttons btn)
    {
        switch(btn)
        {
            case A: return gamepad.a;
            case B: return gamepad.b;
            case X: return gamepad.x;
            case Y: return gamepad.y;
            case BACK: return gamepad.back;
            case START: return gamepad.start;
            case DPAD_UP: return gamepad.dpad_up;
            case DPAD_DOWN: return gamepad.dpad_down;
            case DPAD_LEFT: return gamepad.dpad_left;
            case DPAD_RIGHT: return  gamepad.dpad_right;
            case LEFT_BUMPER: return gamepad.left_bumper;
            case RIGHT_BUMPER: return  gamepad.right_bumper;
            case LEFT_JOYSTICK: return gamepad.left_stick_button;
            case RIGHT_JOYSTICK: return  gamepad.right_stick_button;
            default: return false;
        }
    }

    public double getRawValue(Axes axe)
    {
        switch (axe)
        {
            case LEFT_X: return gamepad.left_stick_x;
            case LEFT_Y: return -gamepad.left_stick_y;
            case RIGHT_X: return gamepad.right_stick_x;
            case RIGHT_Y: return -gamepad.right_stick_y;
            case LEFT_TRIGGER: return gamepad.left_trigger;
            case RIGHT_TRIGGER: return gamepad.right_trigger;
            default: return 0.0;
        }
    }

    public boolean getValue(Buttons btn)
    {
        return valueMap.get(btn);
    }

    public double getValue(Axes axe)
    {
        return getRawValue(axe);
    }

    void update()
    {
        for(Buttons btn: Buttons.values())
        {
            if(typeMap.get(btn) == PressType.PRESS)
            {
                valueMap.put(btn, getRawValue(btn));
            }
            else if(typeMap.get(btn) == PressType.TOGGLE)
            {
                Toggle tgl = toggleMap.get(btn);
                boolean val = tgl.query(getRawValue(btn));
                valueMap.put(btn, val);
            }
        }
    }
}
