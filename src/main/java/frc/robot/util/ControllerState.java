package frc.robot.util;

import java.util.HashMap;

public class ControllerState {

    private HashMap<TeleopMethods, ButtonState> methods = new HashMap<TeleopMethods, ButtonState>();

    public void addMethod(TeleopMethods name) {
        methods.put(name, new ButtonState());
    }

    public ButtonState getMethodState(TeleopMethods method) {
        return methods.get(method);
    }

    public void changeMethodState(TeleopMethods method) {
        ButtonState state = methods.get(method);

        state.state = !state.state;
        methods.put(method, state);
    }
}
