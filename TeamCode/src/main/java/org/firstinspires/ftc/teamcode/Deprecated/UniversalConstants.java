package org.firstinspires.ftc.teamcode.Deprecated;


import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by Ethan Schaffer on 7/3/2017.
 */

public class UniversalConstants {
    public static final double robotHorizontalOffset = 0;
    public static final double robotVerticalOffset = 10;
    public static final double robotFrontOffset = 5;
    public static final double millimetersPerInch = 25.4;

    public static final double degreesPerRadian = 2*Math.PI/360;

    public static final double addToHigherSide = .02;
    public static final double subtractFromLowerSide = .01;

    public static final String leftFrontDrive = "lf";
    public static final String leftBackDrive = "lb";
    public static final String rightFrontDrive = "rf";
    public static final String rightBackDrive = "rb";
    public static final String vuforiaLicenceKey = "ASW6AVr/////AAAAGcNlW86HgEydiJgfyCjQwxJ8z/aUm0uGPANypQfjy94MH3+UHpB" +
            "60bep2E2CIpQCtDevYkE3I9xx1nrU3d9mxfoelhGARuvw7GBwTSjMG0GDQbuSgWGZ1X1IVW35MjOoeg57y/IJGCosxEGz" +
            "J0VHTFmKLkPoGCHQysZ2M2d8AVQDyG+PobNjbYQeC16TZJ7SJyXHr7MJxpj/MKbRwb/bZ1icAvWdrNWiB48dyRjIESk7MewD" +
            "X5ke8X6KEjZkKFiQxbAeCbh3DoxTXVJujcSHAdzncIsFIxLqvh5pX0il9tX+vs+64CUjEbi/HaES7S0q3d3MrVQMCXz77zynqv" +
            "iei9O/4BmYcLw6W7c+Es0sClX/";
    public static final VuforiaLocalizer.CameraDirection camera = VuforiaLocalizer.CameraDirection.FRONT;
    //Back is Selfie Camera

}