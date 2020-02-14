package frc.robot.util;

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
}