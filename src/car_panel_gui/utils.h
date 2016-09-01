#ifndef UTILS_H_
#define UTILS_H_


#include <math.h>


class Utils
{
public:
	static int getFloatPointDigitFromValue(float value)
	{
		return (getFirstDigit(value));
	}

	static int getUnitDigitFromValue(float value)
	{
		int intValue = (int) value;

		return (intValue % 10);
	}

	static int getDozenDigitFromValue(float value)
	{
		int intValue = (int) value;

		return ((intValue % 100 - getUnitDigitFromValue(value)) / 10);
	}

	static int getHundredDigitFromValue(float value)
	{
		int intValue = (int) value;

		return (((intValue % 1000) - getDozenDigitFromValue(value)) - getUnitDigitFromValue(value)) / 100;
	}

	static void getDigitsFromValue(float value, int *hundred, int *dozen, int *unit, int *floatPoint)
	{
		int intValue = (int) floor(value + 0.05);

		(*floatPoint) = getFirstDigit(value);
		(*unit) = intValue % 10;
		(*dozen) = (intValue / 10) % 10;
		(*hundred) = (intValue / 100) % 10;
	}

private:
	static int getFirstDigit(float value)
	{
		int intValue = (int) floor(value + 0.05);

		return ((int) (10.0 * (value - (float) intValue)) % 10);
	}
};

#endif /* UTILS_H_ */
