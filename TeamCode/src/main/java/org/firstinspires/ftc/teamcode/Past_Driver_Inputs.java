package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Vector;
import java.util.HashMap;


/**
 * Created by joshwinebrener on 2/11/17.
 */

public class Past_Driver_Inputs {

    Vector pastAnalogInputs = new Vector();
    Vector pastDigitalInputs = new Vector();
    HashMap<String, Double> analogInputs = new HashMap();
    HashMap<String, Boolean> digitalInputs = new HashMap();

    double joystickX = 0.0;
    double joystickY = 0.0;

    boolean a = false;
    boolean b = false;
    boolean x = false;
    boolean y = false;
    boolean bumperLeft = false;
    boolean bumperRight = false;
    boolean dpadUp = false;
    boolean dpadDown = false;

    int nextVectorSlot = 0;

    public static Gamepad gamepad1;

    Past_Driver_Inputs (Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
    }

    public void recordCurrentInput() {

        //set the hashmap to the current input values
        analogInputs.put("joystickX", (double) gamepad1.left_stick_x);
        analogInputs.put("joystickY", (double) gamepad1.left_stick_y);

        digitalInputs.put("A", gamepad1.a);
        digitalInputs.put("B", gamepad1.b);
        digitalInputs.put("X", gamepad1.x);
        digitalInputs.put("Y", gamepad1.y);
        digitalInputs.put("bumperLeft", gamepad1.left_bumper);
        digitalInputs.put("bumperRight", gamepad1.right_bumper);
        digitalInputs.put("dpadUp", gamepad1.dpad_up);
        digitalInputs.put("dpadDown", gamepad1.dpad_down);

        //record the new hashmap in the vector
        nextVectorSlot = pastAnalogInputs.size() + 1;
        pastAnalogInputs.add(nextVectorSlot, analogInputs);
        pastDigitalInputs.add(nextVectorSlot, digitalInputs);

    }
}
