package org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.*;

import com.qualcomm.robotcore.eventloop.opmode.*;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.ListIterator;

/**
 * Created by Stephen on 2/11/17.
 */

class VarOption<T>
{
    private T value;
    private ListIterator<T> current;
    public ArrayList<T> values = new ArrayList<>();
    public String name = new String();
    private Mutable<T> var;

    VarOption(Mutable<T> variable, String name, T... args)
    {
        var = variable;

        this.name = name;

        for(T arg : args) {
            values.add(arg);
        }

        // We want to set the displayed value to the current state of the var (the variable we are going to set)
        current = values.listIterator();
        while (current.hasNext()) {
            value = current.next();
            if (var == value)
                break;
        }
    }

    public T selected() {
        return value;
    }

    public T next()
    {
        if (current.hasNext())
            value = current.next();
        return value;
    }

    public T prev()
    {
        if (current.hasPrevious())
            value = current.previous();
        return value;
    }

    public void apply()
    {
        var.set(value);
    }

    @Override
    public String toString() {
        return name;
    }
}

public class InteractiveInit {

    Telemetry telemetry;
    Gamepad gamepad1;
    LinearOpMode opMode;
    private boolean interactiveMode = true;

    private Signal sigDpadUp = new Signal();
    private Signal sigDpadDown = new Signal();
    private Signal sigDpadRight = new Signal();
    private Signal sigDpadLeft = new Signal();
    private Signal sigButtonA = new Signal();

    private ArrayList<VarOption<Double>> double_options = new ArrayList<>();
    private ArrayList<VarOption<String>> string_options = new ArrayList<>();
    private ArrayList<VarOption<Boolean>> boolean_options = new ArrayList<>();

    String margin = "       "; // 7 blank spaces
    String cursor = " >> "; // 4 character 'cursor'

    private int cursor_location = 0;

    private int numOptions() {
        return double_options.size() + string_options.size() + boolean_options.size();
    }

    // Applies selected state to 
    private void apply()
    {
        for (VarOption<Double> arg : double_options)
            arg.apply();

        for (VarOption<String> arg : string_options)
            arg.apply();

        for (VarOption<Boolean> arg : boolean_options)
            arg.apply();
    }

    private void nextOption()
    {
        if (cursor_location < double_options.size())
        {
            double_options.get(cursor_location).next();
        }
        else if (cursor_location < double_options.size() + string_options.size())
        {
            string_options.get(cursor_location - double_options.size()).next();
        }
        else if (cursor_location < double_options.size() + string_options.size() + boolean_options.size())
        {
            boolean_options.get(cursor_location - double_options.size() - string_options.size()).next();
        }
    }

    private void prevOption()
    {
        if (cursor_location < double_options.size())
        {
            double_options.get(cursor_location).prev();
        }
        else if (cursor_location < double_options.size() + string_options.size())
        {
            string_options.get(cursor_location - double_options.size()).prev();
        }
        else if (cursor_location < double_options.size() + string_options.size() + boolean_options.size())
        {
            boolean_options.get(cursor_location - double_options.size() - string_options.size()).prev();
        }
    }

    public InteractiveInit(Telemetry telemetry, Gamepad gamepad1, LinearOpMode opMode)
    {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.opMode = opMode;
    }

    public void addDouble(Mutable<Double> var, String name, Double... args)
    {
        double_options.add(new VarOption<>(var, name, args));
    }

    public void addString(Mutable<String> var, String name, String... args)
    {
        string_options.add(new VarOption<>(var, name, args));
    }

    public void addBoolean(Mutable<Boolean> var, String name, Boolean... args)
    {
        boolean_options.add(new VarOption<>(var, name, args));
    }

    /*
     * Display an interactive menu with gamepad controlled cursor
     * based on class variable "PARAMETERS".
     * Should make menu modification simple.
     */
    private void displayMenu() {
        // Format dependent upon class variable 'interactiveMode'.
        int n = numOptions();
        String[] cursorMargin = new String[n];

        if(interactiveMode) {
            for (int i = 0; i < n; ++i) {
                if (cursor_location == i) {
                    cursorMargin[i] = cursor;
                }
                else {
                    cursorMargin[i] = margin;
                }
            }
        } else { // not interactiveMode
            for (int i = 0; i < n; i++) {
                cursorMargin[i] = " "; // remove cursor margin
            }
        }

        telemetry.addData("***"," MENU OPTIONS ***");
        // Iterate over double_options
        for(int i = 0; i < double_options.size(); ++i) {
            telemetry.addData(cursorMargin[i] + double_options.get(i).selected().toString(), double_options.get(i));
        }
        for(int i = 0; i < string_options.size(); ++i) {
            telemetry.addData(cursorMargin[i + double_options.size()] + string_options.get(i).selected().toString(), string_options.get(i));
        }
        for(int i = 0; i < boolean_options.size(); ++i) {
            telemetry.addData(cursorMargin[i + double_options.size() + string_options.size()] + boolean_options.get(i).selected().toString(), boolean_options.get(i));
        }

        if(interactiveMode) {
            telemetry.addLine();
            telemetry.addData("To edit:", "    Use Direction Pad");
            telemetry.addData("To lock:", "    Press  A button");
        } else { // not interactiveMode
            telemetry.addData("INITIALIZATION", "*** LOCKED ***");
        }
    } // displayMenu()

    // menuInputLoop Method.
    // Note: This method contains a while() loop, so will block the program
    // until the menu is locked and exited.
    public void menuInputLoop() {
        while(interactiveMode) {
            displayMenu();
            telemetry.update();
            updateInputs();
        }
        apply();
        displayMenu(); // Display 'locked' version of menu before exit.
        telemetry.update();
    } // menuInputLoop()

    /*
     * Take gamepad inputs and
     * modify parameters accordingly.
     */
    private void updateInputs() {
        // Inputs are updated using the gamepad controls.
        // Loop exits if gamepad not detected or opMode started.
        if(gamepad1 != null && !opMode.opModeIsActive()) {
            if (sigDpadDown.risingEdge(gamepad1.dpad_down)) {
                ++cursor_location;
                if (cursor_location >= numOptions())
                    cursor_location = numOptions() - 1;
            } else if (sigDpadUp.risingEdge(gamepad1.dpad_up)) {
                --cursor_location;
                if (cursor_location < 0)
                    cursor_location = 0;
            } else if (sigDpadRight.risingEdge(gamepad1.dpad_right)) {
                nextOption();
            } else if (sigDpadLeft.risingEdge(gamepad1.dpad_left)) {
                prevOption();
            } else if (sigButtonA.risingEdge(gamepad1.a)) {
                interactiveMode = false;
            }
        }
        else { // No gamepad detected or opMode started.
            interactiveMode = false;
        }

    } // updateInputs()
}
