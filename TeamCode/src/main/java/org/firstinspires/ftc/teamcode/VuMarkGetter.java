package org.firstinspires.ftc.teamcode;

import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


public class VuMarkGetter {
    public VuforiaTrackables getVuforia() {
        return getVuforia(true);
    }

    public VuforiaTrackables getVuforia(boolean showCameraView) {

        VuforiaLocalizer.Parameters parameters;
        if (showCameraView) {

            parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
            // Makes the Vuforia view appear on the phone screen
            // Can remove the R.id.cameraMonitorViewId to save battery or whatever.
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.vuforiaLicenseKey = "ASW6AVr/////AAAAGcNlW86HgEydiJgfyCjQwxJ8z/aUm0uGPANypQfjy94MH3+UHpB" +
                "60bep2E2CIpQCtDevYkE3I9xx1nrU3d9mxfoelhGARuvw7GBwTSjMG0GDQbuSgWGZ1X1IVW35MjOoeg57y/IJGCosxEGz" +
                "J0VHTFmKLkPoGCHQysZ2M2d8AVQDyG+PobNjbYQeC16TZJ7SJyXHr7MJxpj/MKbRwb/bZ1icAvWdrNWiB48dyRjIESk7MewD" +
                "X5ke8X6KEjZkKFiQxbAeCbh3DoxTXVJujcSHAdzncIsFIxLqvh5pX0il9tX+vs+64CUjEbi/HaES7S0q3d3MrVQMCXz77zynqv" +
                "iei9O/4BmYcLw6W7c+Es0sClX/";
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        // Can also use a teapot.

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);
        // We only need one vision target for this year.

        VuforiaTrackables trackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        trackables.setName("Vision Targets");
        return trackables;
    }

    public RelicRecoveryVuMark getPattern(VuforiaTrackables vuforiaTrackables){
        VuforiaTrackable target = vuforiaTrackables.get(0);
        return RelicRecoveryVuMark.from(target);
    }

    public DistanceOffsets getOffset(VuforiaTrackables vuforiaTrackables) {
        VuforiaTrackable target = vuforiaTrackables.get(0);
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) target.getListener()).getPose();
        RelicRecoveryVuMark type = RelicRecoveryVuMark.from(target);
        if (pose != null) {
            double distanceLeftRight = pose.getTranslation().get(0);
            double distanceUpDown = pose.getTranslation().get(1);
            double distanceForwardsBackwards = Math.abs(pose.getTranslation().get(2));
            return new DistanceOffsets(distanceForwardsBackwards, distanceLeftRight, distanceUpDown, type);
        }
        return null;
        //Found nothing
    }

    public class DistanceOffsets {
        public double distance, horizontal, vertical;
        public boolean foundValues;
        public RelicRecoveryVuMark vuMarkType;

        DistanceOffsets(double forwardsBack, double leftRight, double upDown, RelicRecoveryVuMark type) {
            foundValues = true;
            distance = forwardsBack;
            horizontal = leftRight;
            vertical = upDown;
            vuMarkType = type;
        }

        DistanceOffsets() {
            this.foundValues = false;
            distance = horizontal = vertical = 0;
            vuMarkType = RelicRecoveryVuMark.UNKNOWN;
        }
    }
}