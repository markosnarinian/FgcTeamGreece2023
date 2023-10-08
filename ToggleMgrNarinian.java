// TODO: Add comments
package org.firstinspires.ftc.teamcode;

public class ToggleMgrNarinian {
    private int delay;
    private double runtimeAtLastPress;
    private boolean acceptButtonPress;
    private boolean hasBeenFalseSinceLastPress = true;

    public ToggleMgrNarinian(int buttonBlockTime) {
        delay = buttonBlockTime;
        runtimeAtLastPress = - (delay + 1);
    }

    public boolean buttonPress(boolean buttonState, double runtime) {
        acceptButtonPress = false;
        if (buttonState && runtime >= runtimeAtLastPress + delay && hasBeenFalseSinceLastPress) {
            runtimeAtLastPress = runtime;
            hasBeenFalseSinceLastPress = false;
            acceptButtonPress = true;
        }

        if (!buttonState && !hasBeenFalseSinceLastPress) {
            hasBeenFalseSinceLastPress = true;
        }

        return acceptButtonPress;
    }
}