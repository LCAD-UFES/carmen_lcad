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
		int intValue = (int) value;

		(*floatPoint) = getFirstDigit(value);
		(*unit) = intValue % 10;
		(*dozen) = (intValue % 100 - (*unit)) / 10;
		(*hundred) = (((intValue % 1000) - *dozen) - *unit) / 100;
	}

private:
	static int getFirstDigit(float f)
	{
		f = modff(f, &f);

		int i = (int) (f * pow(10, sizeof(float)));
		int i_aux = 0;

		int k = 0;

		while (i >= 10)
		{
			i_aux = i % 10;
			i /= 10;

			if (i_aux >= 5)
			{
				i++;
			}

			k++;
		}

		return k < 3 ? 0 : i;
	}
};

#endif /* UTILS_H_ */
