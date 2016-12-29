package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.TelemetryImpl;

import java.util.ArrayList;

/**
 * Created by Ashley on 12/22/2016.
 * Used within an opmode's initialization to allow user to modify
 * initialization parameters before they are locked in.
 *
 * My Goal was to make customizing this menu as easy as possible, but I think
 * that this attempt has fallen a bit short.
 *
 * There are two particularly rough spots.
 * 1) Because the "Enum[] menuOptions" array has values, not references, the changes
 * made to this array by the updateInputs() method do not directly change the various
 * class enum objects, such as team, position, launchControl, etc. However, it is desireable that
 * these named properties be updated since that makes this class easier to use as a black box.
 * To synchronize these named enum objects with the menuOptions array, the updateOutputs()
 * method must be called. Unfortunately, this method must be modified if new menu entries
 * are added, or if their order is changed.
 *
 * 2) There isn't yet a clear way to have an integer menu item.  It would be nice to have
 * a menu item which specifies max launch speed in RPM's, and can be adjusted within
 * preset bounds by preset increments, but it isn't clear how to do this with the current
 * construction. "Enum[] menuOptions" won't handle a double or int.
 *
 * Note: Beacon Goal calculation in updateOutpus() needs to be implemented.
 *
 *
 */

public class InteractiveInit {


    // *** Enabling Code *** ///
    Telemetry telemetry;
    Gamepad gamepad1;
    LinearOpMode opMode;
    private boolean interactiveMode = true;
    private double cursorSpace = 4; // Margin for cursor
    private Signal sigDpadUp = new Signal();
    private Signal sigDpadDown = new Signal();
    private Signal sigDpadRight = new Signal();
    private Signal sigDpadLeft = new Signal();
    private Signal sigButtonA = new Signal();
    public enum BeaconTarget { GEAR, TOOL, LEGO, WHEEL}
    // public output properties
    public double startX = 0;
    public double startY = 0;
    public double startHeading = 0;
    public ArrayList<Double> goalX = new ArrayList<>();
    public ArrayList<Double> goalY = new ArrayList<>();
    public ArrayList<Double> goalHeading = new ArrayList<>();
    public double startDelaySec = 0;
    public ArrayList<Enum> beaconTargets = new ArrayList<>();


    /**
     *  Note: Any changes to this code block must be reflected in the updateOutputs() method.
     */
    // *** PARAMETERS/MENU SETUP *** //

    // Parameter List
    private enum Menu { Team, Position, LaunchControl, BeaconGoal, StartDelay, DebugMode}

    // Parameter options
    enum Team { RED_TEAM, BLUE_TEAM}
    enum Position { LEFT, RIGHT, ORIGIN}
    enum LaunchControl { PID, POWER} //RUN_USING_ENCODER or RUN_WITHOUT_ENCODER
    enum BeaconGoal { NEAR, FAR, BOTH}
    enum StartDelay { ZERO, FOUR, EIGHT, TWELVE} // Integer numeric would be nice!
    enum DebugMode { ON, OFF}


    // Initial values
    Menu menu = Menu.Team; // Which menu item to change first?

    Team team = Team.RED_TEAM;
    Position position = Position.LEFT;
    LaunchControl launchControl = LaunchControl.PID;
    BeaconGoal beaconGoal = BeaconGoal.NEAR;
    StartDelay startDelay = StartDelay.ZERO;
    DebugMode debugMode = DebugMode.ON;

    // "menuOptions" must match "enum Menu" and updateOutputs()
    Enum[] menuOptions = {team, position, launchControl, beaconGoal, startDelay, debugMode};

    // *** END PARAMETERS/MENU SETUP *** //


    // Class Constructor
    public InteractiveInit(Telemetry telemetry, Gamepad gamepad1, LinearOpMode opMode) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.opMode = opMode;
    }


    // menuInputLoop Method.
    // Note: This method contains a while() loop, so will block the program
    // until the menu is locked and exited.
    public void menuInputLoop() {
        while(interactiveMode) {
            displayMenu();
            telemetry.update();
            updateInputs();
            updateOutputs(); // Update output numbers.
        }
        displayMenu(); // Display 'locked' version of menu before exit.
        telemetry.update();
    } // menuInputLoop()


    /*
     * Display an interactive menu with gamepad controlled cursor
     * based on class variable "PARAMETERS".
     * Should make menu modification simple.
     */
    void displayMenu() {
        // Format dependent upon class variable 'interactiveMode'.
        String margin = "       "; // 7 blank spaces
        String cursor = " >> "; // 4 character 'cursor'
        int numOptions = Menu.values().length;
        String[] cursorMargin = new String[numOptions];
        if(interactiveMode) {
            for (int i = 0; i < numOptions; i++) {
                if (menu.ordinal() != i) {
                    cursorMargin[i] = margin;
                } else {
                    cursorMargin[i] = cursor;
                }
            }
        } else { // not interactiveMode
            for (int i = 0; i < numOptions; i++) {
                cursorMargin[i] = " "; // remove cursor margin
            }
        }

        telemetry.addData("***"," MENU OPTIONS ***");
        // Iterate over
        for(int i=0; i<Menu.values().length; i++) {
            telemetry.addData(cursorMargin[i] + Menu.values()[i].toString(), menuOptions[i]);
        }
        if(interactiveMode) {
            telemetry.addLine();
            telemetry.addData("To edit:", "    Use Direction Pad");
            telemetry.addData("To lock:", "    Press  A button");
        } else { // not interactiveMode
            telemetry.addData("INITIALIZATION", "*** LOCKED ***");
        }

    } // displayMenu()


    /*
     * Take gamepad inputs and
     * modify parameters accordingly.
     */
    private void updateInputs() {
        // Inputs are updated using the gamepad controls.
        // Loop exits if gamepad not detected or opMode started.
        if(gamepad1 != null && !opMode.opModeIsActive()) {
            if (sigDpadDown.risingEdge(gamepad1.dpad_down)) {
                menu = EnumHelper.getNext(menu);
            } else if (sigDpadUp.risingEdge(gamepad1.dpad_up)) {
                menu = EnumHelper.getPrevious(menu);
            } else if (sigDpadRight.risingEdge(gamepad1.dpad_right)) {
                // getNext
                menuOptions[menu.ordinal()] = EnumHelper.getNext(menuOptions[menu.ordinal()]);
            } else if (sigDpadLeft.risingEdge(gamepad1.dpad_left)) {
                // getPrev
                menuOptions[menu.ordinal()] = EnumHelper.getPrevious(menuOptions[menu.ordinal()]);
            } else if (sigButtonA.risingEdge(gamepad1.a)) {
                interactiveMode = false;
            }
        }
        else { // No gamepad detected or opMode started.
            interactiveMode = false;
        }

    } // updateInputs()

    private void updateOutputs() {
        // Update named enum variables from array
        // This code is messy and difficult to maintain. Sadness.
        team = (Team)menuOptions[0];
        position = (Position)menuOptions[1];
        launchControl = (LaunchControl)menuOptions[2];
        beaconGoal = (BeaconGoal)menuOptions[3];
        startDelay = (StartDelay)menuOptions[4];
        debugMode = (DebugMode)menuOptions[5];


        // Calculate starting position
        int startSpacing = 550; // mm from center to center of team robots. About 22"
        int startOffset = 0; // Distance from team center to wall center.
        if (position == Position.ORIGIN){
                startX = 0;
                startY = 0;
                startHeading = 0;
        } else {
            switch (team) {
                case RED_TEAM:
                    switch (position) {
                        case LEFT:
                            startX = -225;
                            startY = -1800;
                            startHeading = 90;
                            break;
                        case RIGHT:
                            startX = 225;
                            startY = -1800;
                            startHeading = 90;
                            break;
                    }
                    break; // end RED_TEAM
                case BLUE_TEAM:
                    switch (position) {
                        case LEFT:
                            startX = 1800;
                            startY = -225;
                            startHeading = 180;
                            break;
                        case RIGHT:
                            startX = 1800;
                            startY = 225;
                            startHeading = 180;
                            break;
                    }
                    break; // end BLUE_TEAM
            }
        }


        // Calculate Beacon Goal Locations
            // Reset old values
            goalX.clear();
            goalY.clear();
            goalHeading.clear();
            beaconTargets.clear();
        switch (team) {
            case RED_TEAM:
                switch (beaconGoal) {
                    case NEAR:
                        // Gears
                        goalX.add((double) VisualNavigation.gearTargetLocationOnField.getTranslation().get(0));
                        goalY.add((double) VisualNavigation.gearTargetLocationOnField.getTranslation().get(1));
                        goalHeading.add(getRobotHeading(VisualNavigation.gearTargetLocationOnField));
                        beaconTargets.add(BeaconTarget.GEAR);
                        break; // end NEAR
                    case FAR:
                        // Tools
                        goalX.add((double) VisualNavigation.toolTargetLocationOnField.getTranslation().get(0));
                        goalY.add((double) VisualNavigation.toolTargetLocationOnField.getTranslation().get(1));
                        goalHeading.add(getRobotHeading(VisualNavigation.toolTargetLocationOnField));
                        beaconTargets.add(BeaconTarget.TOOL);
                        break; // end FAR
                    case BOTH:
                        // Gears and Tools
                        goalX.add((double) VisualNavigation.gearTargetLocationOnField.getTranslation().get(0));
                        goalY.add((double) VisualNavigation.gearTargetLocationOnField.getTranslation().get(1));
                        goalHeading.add(getRobotHeading(VisualNavigation.gearTargetLocationOnField));
                        beaconTargets.add(BeaconTarget.GEAR);
                        goalX.add((double) VisualNavigation.toolTargetLocationOnField.getTranslation().get(0));
                        goalY.add((double) VisualNavigation.toolTargetLocationOnField.getTranslation().get(1));
                        goalHeading.add(getRobotHeading(VisualNavigation.toolTargetLocationOnField));
                        beaconTargets.add(BeaconTarget.TOOL);
                        break; // end BOTH
                }
                break; // end RED_TEAM
            case BLUE_TEAM:
                switch (beaconGoal) {
                    case NEAR:
                        // Wheels
                        goalX.add((double) VisualNavigation.wheelTargetLocationOnField.getTranslation().get(0));
                        goalY.add((double) VisualNavigation.wheelTargetLocationOnField.getTranslation().get(1));
                        goalHeading.add(getRobotHeading(VisualNavigation.wheelTargetLocationOnField));
                        beaconTargets.add(BeaconTarget.WHEEL);
                        break; // end NEAR
                    case FAR:
                        // Legos
                        goalX.add((double) VisualNavigation.legoTargetLocationOnField.getTranslation().get(0));
                        goalY.add((double) VisualNavigation.legoTargetLocationOnField.getTranslation().get(1));
                        goalHeading.add(getRobotHeading(VisualNavigation.legoTargetLocationOnField));
                        beaconTargets.add(BeaconTarget.LEGO);
                        break; // end FAR
                    case BOTH:
                        // Wheels and Legos
                        goalX.add((double) VisualNavigation.wheelTargetLocationOnField.getTranslation().get(0));
                        goalY.add((double) VisualNavigation.wheelTargetLocationOnField.getTranslation().get(1));
                        goalHeading.add(getRobotHeading(VisualNavigation.wheelTargetLocationOnField));
                        beaconTargets.add(BeaconTarget.WHEEL);
                        goalX.add((double) VisualNavigation.legoTargetLocationOnField.getTranslation().get(0));
                        goalY.add((double) VisualNavigation.legoTargetLocationOnField.getTranslation().get(1));
                        goalHeading.add(getRobotHeading(VisualNavigation.legoTargetLocationOnField));
                        beaconTargets.add(BeaconTarget.LEGO);
                        break; // end BOTH
                }
                break; // end BLUE_TEAM
        } // switch team



        // Calculate startDelay int
        switch (startDelay) {
            case ZERO:
                startDelaySec = 0;
                break;
            case FOUR:
                startDelaySec = 4;
                break;
            case EIGHT:
                startDelaySec = 8;
                break;
            case TWELVE:
                startDelaySec = 12;
                break;
            default:
                startDelaySec = 0;
                break;
        }

    } // updateOutputs()

    double getRobotHeading(OpenGLMatrix targetOpenGLMatrix) {
        // Returns the robot heading required to face into (oposite) of the
        // Specified visual target normal axis.
        double visualTargetOrientation = (double) Orientation.getOrientation(
                targetOpenGLMatrix, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        // Rotate into Robot's perspective:
        return 90 + visualTargetOrientation;
    } // getRobotHeading()

} // InteractiveInit class



// Enum increment helper
class EnumHelper {
     static <E extends Enum<E>> E getNext(E enumObject) {
         int oldOrdinal = enumObject.ordinal();
         int enumLength = enumObject.getDeclaringClass().getEnumConstants().length;
         int newOrdinal = (oldOrdinal + 1) % enumLength;
         E newEnum = (enumObject.getDeclaringClass().getEnumConstants()[newOrdinal]);
        return newEnum;
    }

    static <E extends Enum<E>> E getPrevious(E enumObject) {
        int oldOrdinal = enumObject.ordinal();
        int enumLength = enumObject.getDeclaringClass().getEnumConstants().length;
        int newOrdinal = (oldOrdinal - 1 + enumLength) % enumLength;
        E newEnum = (enumObject.getDeclaringClass().getEnumConstants()[newOrdinal]);
        return newEnum;
    }

    // Example from stackOverflow
    static <E extends Enum<E>> E[] getValuesForEnum(Class<E> clazz) {
        return clazz.getEnumConstants();
    }

} //EnumHelper class