package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

/** if our code is colonial America, this class is Rhode Island */
public class Util {

	/**
	 * Trims int within a range
	 * @param input int before truncation
	 * @param lowLimit lowest allowable int
	 * @param highLimit highest allowable int
	 * @return truncated int
	 */
	public static int truncateInt(int input, int lowLimit, int highLimit) {
		if (input < lowLimit) { input = lowLimit; }
		if (input > highLimit) { input = highLimit; }
		return input;
	}

	/**
	 * Converts celsius temperature to fahrenheit temperature
	 * @param celsius temperature in celsius degrees
	 * @return temperature in fahrenheit degrees
	 */
	public static double celsiusToFahrenheit(double celsius) {
		return (celsius * (9/5)) + 32;
	}

	/**
	 *	Kind of self explanatory, but with some spice
	 *	Use this mainly as a get method to retrieve values the user types into the smart dash
	 *		(the 'Set' part is only in case the value doesn't exist, backup is a default to use and set if it isn't there)
	 */
	static Preferences pref = Preferences.getInstance();
	public static double getAndSetDouble(String key, double backup) {
		if(!pref.containsKey(key)) pref.putDouble(key, backup);
		return pref.getDouble(key, backup);
	}

	/**
	 * Gets boolean value from Prefs, or sets it to backup if it doesn't exist
	 * @param key    The name of the bool to grab
	 * @param backup The backup value to use if the bool doesn't exist
	 * @return       The value of the boolean "Key"
	 */
	public static boolean getAndSetBoolean(String key, boolean backup) {
		if(!pref.containsKey(key)) pref.putBoolean(key, backup);
		return pref.getBoolean(key, backup);
	}

	/**
     * Really stupid but needed to round a double to n places
     * @param value  original value
     * @param places how many values after decimal point
     * @return       rounded value
     */
    public static double roundTo(double value, int places) {
        double val = value;
        val *= Math.pow(10, places);
        val = Math.round(val);
        val /= Math.pow(10, places);
        return val;
	}

	/**
	 * Converts a force in lbf to Newtons (N)
	 * @param lbf A value of force (lbf)
	 * @return A value of force in Newtons
	 */
	public static double poundForceToNewtons(double lbf) {
		return lbf * 4.44822;
	}

	/**
	 * Converts a force in Newtons to Pound-Force (lbf)
	 * @param n A value of force in Newtons.
	 * @return A value of force in Pounds (lbf).
	 */
	public static double newtonsToPoundForce(double n) {
		return n / 4.44822;
	}

	/**
	 * Converts a weight in lbf to kg.
	 * @param lbf Weight in lbf (this is the readout that you get from any scale that deals in pounds)
	 * @return Mass in kg
	 */
	public static double weightLBFToMassKG(double lbf) {
		return poundForceToNewtons(lbf) / 9.81; //convert to newtons and divide by the gravitational field strength.
	}

	/**
	 * Calculates a weight in Pounds given a mass in kilograms.
	 * @param kg A mass in kilograms (the value you get from a balance)
	 * @return Weight in pounds.
	 */
	public static double massKGToWeightLBF(double kg) {
		return newtonsToPoundForce(kg * 9.81);
	}

	/**
	 * Converts a value in inches to meters.
	 * @param in A value of inches.
	 * @return A value of meters.
	 */
	public static double inchesToMeters(double in) {
		return in * 0.0254;
	}

	/**
	 * Converts a value in meters to inches.
	 * @param m A value of meters.
	 * @return A value of inches.
	 */
	public static double metersToInches(double m) {
		return m / 0.0254;
	}
	
	/**
	 * Checks if a controller exists.
	 * @param index The index of the controller (same you would use to define the Joystick)
	 * @return      True if the controller exists, false otherwise.
	 */
	public static boolean controllerExists(int index) {
		return DriverStation.getInstance().getJoystickName(index) != "";
	}

	public static double closestToZero(double[] set) {
		double least = Double.MAX_VALUE;
		for(int i=0; i<set.length; i++) {
			if(Math.abs(set[i]) < Math.abs(least)) {
				least = set[i];
			}
		}

		return least;
	}

	/**
	 * Gets the angle that the robot needs to turn through to acheive a heading.
	 * @param angle   The angle of the robot
	 * @param heading The desired heading to be acheived.
	 * @return        The angle that the robot needs to turn to have its desired heading.
	 */
	public static double getAngleToHeading(double angle, double heading) {
		double angle1 = heading - angle;         //angle to heading without crossing 0
		double angle2 = angle1 - 360;
		double angle3 = angle1 + 360;

		return closestToZero(new double[] {angle1, angle2, angle3});
	}

	/**
	 * This method makes an angle acute by changing where it's horizontal reference point is.
	 * @param angle The angle to make acute.
	 * @return An angle who's absolute value is less than or equal to 90. 
	 */
	public static double getAcuteSuppliment(double angle) {
		boolean isNegative = angle < 0;
		double acuteAngle = Math.abs(angle) % 180;
		acuteAngle = 180 - acuteAngle;

		if(isNegative) {
			acuteAngle *= -1;
		}

		return acuteAngle;
	}

	/**
	 * Tests the equality of two values, then prints and returns the result.
	 * Print string will be displayed on RioLog as an error reading: "Assertion [assertionID] SUCCEEDED/FAILED."
	 * @param assertionName The informational name of the assertion. Will be used in the printout.
	 * @param item1       The first item to test.
	 * @param item2       The second item to test.
	 * @return            True if item1 equals item2. False otherwise.
	 */
	public static boolean assertEquals(String assertionName, Object item1, Object item2) {
		boolean success = item1.equals(item2);
		String message = "Assertion " + assertionName + " " + (success ? "SUCCEEDED" : "FAILED") + "." + " Item 1: " + item1.toString() + " | Item 2: " + item2.toString();
		DriverStation.reportError(message, false);
		return success;
	}
}