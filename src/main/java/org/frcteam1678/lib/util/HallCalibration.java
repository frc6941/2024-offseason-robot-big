package org.frcteam1678.lib.util;

public class HallCalibration {

    /*
     * The max and min raw sensor values of when the hall sensor reads true, as seen
     * so far. When calibration is complete it is approximately the top and bottom
     * of the magnet's range.
     */
    double mMaxHallTrue = 0;
    double mMinHallTrue = 0;

    // The max and min raw sensor values that have been read
    double mMaxOverall = 0;
    double mMinOverall = 0;

    // Whether it is the first time running Update()
    boolean mFirstTime = true;

    // Whether the hall sensor has ever been triggered
    boolean mMagnetFound = false;
    boolean mCalibrated = false;

    // The offset is the value *added* to the raw sensor values
    double offset = 0;

    // The value that should be returned when at the center of the magnet
    double mMagnetPosition;

    /**
     * The hall calibration instance class
     * @param resetPosition the hall effect sensor's reset position
     */
    public HallCalibration(double resetPosition) {
        mMagnetPosition = resetPosition;

    }

    public double update(double mainSensorValue, boolean hallValue) {
        if (hallValue) {

            /*
             * Update the max and min values for when the hall sensor is triggered. Set them
             * to the current value if it is the first time seeing the magnet.
             */
            if (mainSensorValue > mMaxHallTrue || !mMagnetFound) {
                mMaxHallTrue = mainSensorValue;
            }
            if (mainSensorValue < mMinHallTrue || !mMagnetFound) {
                mMinHallTrue = mainSensorValue;
            }
            mMagnetFound = true;
        }

        /*
         * Update the max and min overall values. Set to the the current value if it is
         * the first time running Update()
         */
        if (mainSensorValue > mMaxOverall || mFirstTime) {
            mMaxOverall = mainSensorValue;
        }
        if (mainSensorValue < mMinOverall || mFirstTime) {
            mMinOverall = mainSensorValue;
        }

        /*
         * Return the best estimate known. If the magnet is not found or the edges of
         * the magnet's range have not been reached, there is no best estimate. In the
         * event that mCalibrated is true, do not set it to false even if the //
         * condition is not currently met, as that could reset any portions of code that
         * assume calibration is complete.
         */
        if ((mMagnetFound && mMaxOverall > mMaxHallTrue && mMinOverall < mMinHallTrue) || mCalibrated) {

            /*
             * The center of the magnet's range is mMagnetPosition, so the offset if
             * mMagnetPosition - the raw value of the center of the magnet
             */
            offset = mMagnetPosition - (mMaxHallTrue + mMinHallTrue) / 2;
            mCalibrated = true;
        }

        mFirstTime = false;
        return mainSensorValue + offset;
    }

    /**
     * Checks if the hall effect is calibrated
     * @return if it is calibrated
     */
    public boolean isCalibrated() {
        return mCalibrated;
    }

    /**
     * Gets the hall offset
     * @return the current offset
     */
    public double getOffset() {
        return offset;
    }
}
