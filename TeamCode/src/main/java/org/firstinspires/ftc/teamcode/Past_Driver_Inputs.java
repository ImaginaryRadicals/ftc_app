package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Vector;
import java.util.HashMap;


/**
 * Created by joshwinebrener on 2/11/17.
 *
 * This class has two methods: one that records the input of the driver, and one that plays it back
 * by changing the values of multiple variables in a hash table.
 *
 * The hash table itself holds all of the relevant driver input for any one moment.  It records all
 * of the gamepad values, as well as the time at which they were input.  This hash table is then
 * stored as a value in a vector, which holds all the previous hash tables representing the input for
 * a certain moment.
 */

public class Past_Driver_Inputs {

    HashMap<String, Object> currentInput = new HashMap();

    Vector<HashMap> pastInputs = new Vector<>();

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
    double lastTime = 0.0;
    double nextTime = 0.0;
    double interval = 0.0;

    public static Gamepad gamepad1;
    public static ElapsedTime runtime;


    Past_Driver_Inputs (Gamepad gamepad1, ElapsedTime runtime) {
        this.gamepad1 = gamepad1;
        this.runtime = runtime;
    }

    public void recordCurrentInput() {

        //set the hashmap to the current input values
        currentInput.put("joystickX", gamepad1.left_stick_x);
        currentInput.put("joystickY", gamepad1.left_stick_y);

        currentInput.put("A", gamepad1.a);
        currentInput.put("B", gamepad1.b);
        currentInput.put("X", gamepad1.x);
        currentInput.put("Y", gamepad1.y);
        currentInput.put("bumperLeft", gamepad1.left_bumper);
        currentInput.put("bumperRight", gamepad1.right_bumper);
        currentInput.put("dpadUp", gamepad1.dpad_up);
        currentInput.put("dpadDown", gamepad1.dpad_down);

        currentInput.put("time", runtime);

        nextVectorSlot = pastInputs.size() + 1;
        pastInputs.add(nextVectorSlot, currentInput);
    }

    public void playBackPastInputs() {

        for (int i = 0; i < currentInput.size(); i++) {

            joystickX = (double) pastInputs.get(i).get("joystickX");
            joystickY = (double) pastInputs.get(i).get("joystickY");
            a = (boolean) pastInputs.get(i).get("A");
            b = (boolean) pastInputs.get(i).get("B");
            x = (boolean) pastInputs.get(i).get("X");
            y = (boolean) pastInputs.get(i).get("Y");
            bumperLeft = (boolean) pastInputs.get(i).get("bumperLeft");
            bumperRight = (boolean) pastInputs.get(i).get("bumperRight");
            dpadUp = (boolean) pastInputs.get(i).get("dpadUp");
            dpadDown = (boolean) pastInputs.get(i).get("dpadDown");

            lastTime = (double) pastInputs.get(i-1).get("time");
            nextTime = (double) pastInputs.get(i).get("time");
            interval = lastTime - nextTime;

            //wait for the interval
            //we may want this interval to be slightly shorter to account for lag
            try {
                Thread.sleep((long) interval);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

        }
    }
}
