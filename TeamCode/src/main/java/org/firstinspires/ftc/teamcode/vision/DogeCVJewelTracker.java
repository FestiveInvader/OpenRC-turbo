package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.Autonomous.JewelDetector;
import org.opencv.core.Mat;

/**
 * Created by Eric on 4/1/2018.
 */
public class DogeCVJewelTracker extends Tracker {
    private JewelDetector jewelDetector;

    public DogeCVJewelTracker(JewelDetector jewelDetector) {
        this.jewelDetector = jewelDetector;
    }

    @Override
    public void init(org.firstinspires.ftc.teamcode.vision.VisionCamera camera) {
        jewelDetector.enable();

    }

    @Override
    public void processFrame(Mat frame, double timestamp) {
        jewelDetector.processFrame(frame, null);
    }

    @Override
    public void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {
    }
    public JewelDetector.JewelOrder getLastOrder(){
        return jewelDetector.getLastOrder();
    }
}
