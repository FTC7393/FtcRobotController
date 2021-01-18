package ftc.electronvolts.util;
public class AnalogInputEdgeDetector implements InputExtractor<Boolean> {
    private Boolean currentValue = null;
    private Boolean previousValue = null;
    private Double currentInput = null;
    private InputExtractor<Double> extractor;
    private final AlivenessTester at;
    private final double lowThreshold;
    private final double highThreshold;
    private final boolean invert;

    /**
     * @param extractor the InputExtractor to do edge detection on
     */
    public AnalogInputEdgeDetector(InputExtractor<Double> extractor, double lowThreshold, double highThreshold, boolean invert) {
        this(extractor,lowThreshold, highThreshold, invert, AlivenessTester.ALWAYS);
    }

    /**
     * @param extractor the InputExtractor to do edge detection on
     */
    public AnalogInputEdgeDetector(InputExtractor<Double> extractor, double lowThreshold, double highThreshold, boolean invert, AlivenessTester at) {
        this.extractor = extractor;
        this.at = at;
        this.lowThreshold = lowThreshold;
        this.highThreshold = highThreshold;
        this.invert = invert;
    }

    /**
     * update the current and previous value of the input
     *
     * @return the current value of the input
     */
    public boolean update() {
        // if this is the first call to update()
        if (at.isAlive()) {
            if (currentValue == null) {
                // set currentValue and previousValue to the reading so no edges are
                // triggered
                currentInput = extractor.getValue();
                if(invert){
                    currentInput *= -1;
                }

                previousValue = false;
                currentValue = false;
            } else {
                previousValue = currentValue;

                currentInput = extractor.getValue();
                if(invert){
                    currentInput *= -1;
                }

                if (currentValue == false) {
                    if(currentInput > highThreshold ){
                        currentValue = true;
                    }

                }else{
                    if (currentInput < lowThreshold){
                        currentValue = false;
                    }
                }
            }
        }
        return currentValue;
    }

    /**
     * @return whether or not the input is true right now
     */
    @Override
    public Boolean getValue() {
        return currentValue;
    }

    /**
     * @return whether or not the input is true right now
     */
    public boolean isPressed() {
        return currentValue;
    }

    /**
     * @return if the input just turned from false to true
     */
    public boolean justPressed() {
        return currentValue && !previousValue;
    }

    /**
     * @return if the input just turned from true to false
     */
    public boolean justReleased() {
        return !currentValue && previousValue;
    }
}
