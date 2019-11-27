package org.firstinspires.ftc.teamcode;

public class ThunderbotsUtil { public static double ConvertDegreesToInches (double diameter, int degree) {
    // Convert degrees to radians
    double radian = degree * 0.0174;
   return  (diameter/2) * radian;
}
}
