package org.firstinspires.ftc.teamcode.Utilities;

/**
 * Created by Josh Winebrener on 11/25/2016.
 */

public class Signal {
    public boolean lastState     = false;
    public boolean currentState  = false;

    public void updateState(boolean newState) {
        lastState    = currentState;
        currentState = newState;
    }

    public boolean risingEdge(){
        if (lastState == false && currentState == true) {
            return true;
        } else {
            return false;
        }
    }

    public boolean risingEdge(boolean newState) {
        updateState(newState);
        return risingEdge();
    }
}
