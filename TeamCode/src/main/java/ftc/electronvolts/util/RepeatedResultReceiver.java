package ftc.electronvolts.util;

/**
 * This file was made by the electronVolts, FTC team 7393
 *
 * Note: value will be not null before this is ready, be sure to call isReady() so that way you can use the logic in setValue()
 *
 */
public class RepeatedResultReceiver<T> implements ResultReceiver<T> {
    private T value = null;
    private boolean ready = false;
    private int counter = 0;
    private final int numberOfRepetitions;

    public RepeatedResultReceiver(int numberOfRepetitions) {
        this.numberOfRepetitions = numberOfRepetitions;
    }

    @Override
    public boolean isReady() {
        return ready;
    }

    @Override
    public T getValue() {
        return value;
    }

    @Override
    public void setValue(T value) {
        if(this.value == null) {
            this.value = value;
            counter++;
        }
        if(this.value.equals(value)){
            counter++;
        } else {
            counter = 0;
        }
        if(counter >= numberOfRepetitions) {
            ready = true;
        }
        this.value = value;
    }

    @Override
    public void clear() {
        ready = false;
        value = null;
        counter = 0;
    }
}
