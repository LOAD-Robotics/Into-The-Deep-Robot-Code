package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

/**
 * A Public class library of misc nonspecific useful functions!
 * */

public class LOAD_Tools {

    /**
     * Simple vector rotation that takes an input array and will rotate
     * x and y by transform angle and output the new array
     *
     * @param inpArray The array formatted as {x,y,heading}
     * */

    public float[] rotateVector(float[] inpArray) {

        float[] output = new float[3]; // Declare the adjusted output value
        float[] tempPolar = new float[2];  // Declare a temporary polar vector for the maths

        tempPolar[0] = (float) Math.sqrt(Math.pow(inpArray[0],2) + Math.pow(inpArray[1],2)); // Calculate the R portion of our tempPolar vector
        // Calculate the Theta portion of our tempPolar vector and adjust it by our heading
        tempPolar[1] = (float) Math.atan2(inpArray[1],inpArray[0]) + (float) Math.toRadians(-inpArray[2]);

        output[0] = (float) Math.cos(tempPolar[1]) * tempPolar[0]; // Convert adjusted vector back to rectangular and assign it to output x
        output[1] = (float) Math.sin(tempPolar[1]) * tempPolar[0]; // Convert adjusted vector back to rectangular and assign it to output y
        output[2] = inpArray[2]; // keep heading the same as it was

        return output; // output value =D
    }

    /**
     * Takes inputs of the joystick positions,
     * and returns 4 values for each of the 4 motor powers
     *
     * @param inpArray The array formatted as {leftStickX, leftStickY, rightStickX}
     * */

    public float[] robotCentricDriving(float[] inpArray) {
        // Separate out the values for each of the joystick positions
        float X = inpArray[0];
        float Y = -inpArray[1];
        float rX = inpArray[2];

        // Do some calculations in order to obtain a number that is used to ensure
        // that the outputs of this function are constrained to the interval [-1,1]
        float d = (float) JavaUtil.maxOfList(JavaUtil.createListWith((Math.abs(Y) + Math.abs(X) + Math.abs(rX)), 1));

        // Calculate the values for each of the drive motors
        float FL = (((Y + X + rX) / d));
        float BL = (((Y - X + rX) / d));
        float FR = (((Y - X - rX) / d));
        float BR = (((Y + X - rX) / d));

        // Define and assign values to the output variable
        float[] output = new float[4];
        output[0] = FL;
        output[1] = BL;
        output[2] = FR;
        output[3] = BR;

        // Return the calculated motor powers
        return output;
    }

    /**
     * Takes inputs of the joystick positions and robot heading,
     * and returns 4 values for each of the 4 motor powers
     *
     * @param inpArray The array formatted as {leftStickX, leftStickY, rightStickX, robotHeading}
     * */

    public float[] fieldCentricDriving(float[] inpArray) {
        // Separate out the values for each of the joystick positions
        float X = inpArray[0];
        float Y = -inpArray[1];
        float rX = inpArray[2];
        float h = inpArray[3];

        // Perform a vector rotation of the left stick based on the robot's heading
        float[] inputVector = new float[3];
        inputVector[0] = X;
        inputVector[1] = Y;
        inputVector[2] = h;
        float[] correctedVector = rotateVector(inputVector);
        X = correctedVector[0];
        Y = correctedVector[1];

        // Do some calculations in order to obtain a number that is used to ensure
        // that the outputs of this function are constrained to the interval [-1,1]
        float d = (float) JavaUtil.maxOfList(JavaUtil.createListWith((Math.abs(Y) + Math.abs(X) + Math.abs(rX)), 1));

        // Calculate the values for each of the drive motors
        float FL = (((Y + X + rX) / d));
        float BL = (((Y - X + rX) / d));
        float FR = (((Y - X - rX) / d));
        float BR = (((Y + X - rX) / d));

        // Define and assign values to the output variable
        float[] output = new float[4];
        output[0] = FL;
        output[1] = BL;
        output[2] = FR;
        output[3] = BR;

        // Return the calculated motor powers
        return output;
    }

}
