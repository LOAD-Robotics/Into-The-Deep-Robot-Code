package org.firstinspires.ftc.teamcode;

/**
 * A Public class library of misc nonspecific useful functions!
 * */

public class LOAD_Tools {

    /**
     * Simple vector rotation that takes an input array and will rotate
     * x and y by transform angle and output the new array
     *
     * @param inpArray The array formatted as {x,y,rotation_angle}
     * */

    public float[] rotateVector(float[] inpArray) {

        float[] output = new float[3]; // Declare the adjusted output value
        float[] tempPolar = new float[2];  // Declare a temporary polar vector for the maths

        tempPolar[0] = (float) Math.sqrt(Math.pow(inpArray[0],2) + Math.pow(inpArray[1],2)); // Calculate the R portion of our tempPolar vector
        // Calculate the Theta portion of our tempPolar vector and adjust it by our heading
        tempPolar[1] = (float) Math.atan2(inpArray[0],inpArray[1]) + (float) Math.toRadians(inpArray[2]);

        output[0] = (float) Math.cos(tempPolar[1]) * tempPolar[0]; // Convert adjusted vector back to rectangular and assign it to output x
        output[1] = (float) Math.sin(tempPolar[1]) * tempPolar[0]; // Convert adjusted vector back to rectangular and assign it to output y
        output[2] = inpArray[2]; // keep heading the same as it was

        return output; // output value =D
    }

}
