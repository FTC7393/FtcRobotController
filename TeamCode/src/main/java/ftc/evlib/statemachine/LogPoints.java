package ftc.evlib.statemachine;

import java.util.HashMap;
import java.util.Map;

public class LogPoints {
    private static Map<String, Object> map = new HashMap<>();
    public static void setLogPoint(String label, Object value) {
        map.put(label, value);
    }

    public static Object getLogPoint(String label) {
        return map.get(label);
    }

}
