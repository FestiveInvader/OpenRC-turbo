package org.firstinspires.ftc.teamcode.vision;

import android.app.Activity;
import android.widget.FrameLayout;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.CameraCalibration;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;

public class VuforiaCamera extends VisionCamera {
    public static final String TAG = "VuforiaCamera";

    public static final String VUFORIA_LICENSE_KEY = "ASW6AVr/////AAAAGcNlW86HgEydiJgfyCjQwxJ8z/aUm0uGPANypQfjy94MH3+UHpB60bep2E2CIpQCtDevYkE3I9xx1nrU3d9mxfoelhGARuvw7GBwTSjMG0GDQbuSgWGZ1X1IVW35MjOoeg57y/IJGCosxEGzJ0VHTFmKLkPoGCHQysZ2M2d8AVQDyG+PobNjbYQeC16TZJ7SJyXHr7MJxpj/MKbRwb/bZ1icAvWdrNWiB48dyRjIESk7MewDX5ke8X6KEjZkKFiQxbAeCbh3DoxTXVJujcSHAdzncIsFIxLqvh5pX0il9tX+vs+64CUjEbi/HaES7S0q3d3MrVQMCXz77zynqviei9O/4BmYcLw6W7c+Es0sClX/";

    private VuforiaLocalizer vuforia;
    private FrameLayout cameraLayout;
    private OverlayView overlayView;
    private ExecutorService frameConsumerExecutor;

    private class FrameConsumer implements Runnable {
        private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;
        private Mat frame;
        private byte[] frameBuffer;

        private FrameConsumer(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue) {
            this.frameQueue = frameQueue;
        }

        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                // grab frames and process them
                if (!frameQueue.isEmpty()) {
                    VuforiaLocalizer.CloseableFrame vuforiaFrame = null;
                    try {
                        vuforiaFrame = frameQueue.take();
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }

                    if (vuforiaFrame == null) {
                        continue;
                    }

                    for (int i = 0; i < vuforiaFrame.getNumImages(); i++) {
                        Image image = vuforiaFrame.getImage(i);
                        if (image.getFormat() == PIXEL_FORMAT.RGB888) {
                            int imageWidth = image.getWidth(), imageHeight = image.getHeight();
                            ByteBuffer byteBuffer = image.getPixels();
                            if (frameBuffer == null) {
                                frameBuffer = new byte[byteBuffer.capacity()];
                            }
                            byteBuffer.get(frameBuffer);
                            if (this.frame == null) {
                                this.frame = new Mat(imageHeight, imageWidth, CvType.CV_8UC3);

                                if (overlayView != null) {
                                    overlayView.setImageSize(imageWidth, imageHeight);
                                }
                            }
                            this.frame.put(0, 0, frameBuffer);

                            //Imgproc.cvtColor(this.frame, this.frame, Imgproc.COLOR_RGB2BGRA);

                            if (parameters.cameraDirection == VuforiaLocalizer.CameraDirection.FRONT) {
                                Core.flip(this.frame, this.frame, 1);
                            }

                            onFrame(this.frame, vuforiaFrame.getTimeStamp());
                        }
                    }
                    vuforiaFrame.close();
                } else {
                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        }
    }

    public VuforiaCamera() {
        this(new Parameters());
    }

    public VuforiaCamera(Parameters parameters) {
        super(parameters);
    }

    @Override
    public synchronized void addTracker(Tracker tracker) {
        super.addTracker(tracker);

        if (overlayView != null) {
            this.overlayView.addTracker(tracker);
        }
    }

    @Override
    protected void onFrame(Mat frame, double timestamp) {
        super.onFrame(frame, timestamp);

        if (overlayView != null) {
            overlayView.postInvalidate();
        }
    }

    @Override
    protected void doInitialize() {
        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(parameters.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = parameters.cameraDirection;
        vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParams);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1);



        frameConsumerExecutor = ThreadPool.newSingleThreadExecutor("Vuforia frame consumer");
        frameConsumerExecutor.execute(new FrameConsumer(vuforia.getFrameQueue()));
    }

    public VuforiaLocalizer getVuforia() {
        return this.vuforia;
    }

    public void close() {
        if (overlayView != null) {
            appUtil.runOnUiThread(() -> {
                cameraLayout.removeView(overlayView);
                overlayView = null;
            });
        }

        if (frameConsumerExecutor != null) {
            frameConsumerExecutor.shutdownNow();
            frameConsumerExecutor = null;
        }
    }

    @Override
    public Properties getProperties() {
        return new VuforiaProperties();
    }

    private class VuforiaProperties implements Properties {
        @Override
        public double getHorizontalFocalLengthPx(double imageWidth) {
            CameraCalibration cameraCalibration = CameraDevice.getInstance().getCameraCalibration();
            double fov = cameraCalibration.getFieldOfViewRads().getData()[0];
            return (imageWidth * 0.5) / Math.tan(0.5 * fov);
        }
    }
}
