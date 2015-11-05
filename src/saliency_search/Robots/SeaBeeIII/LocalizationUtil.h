#include "Util/MathFunctions.H"
#include "Image/PixelsTypes.H"
#include "Image/Point2D.H"
#include "Image/Point3D.H"
#include "XBox360RemoteControlI.H"
#include <map>
#include <fstream>
#include <sstream>

#ifndef LOCALIZATIONUTIL_H_
#define LOCALIZATIONUTIL_H_

using namespace std;

class ParamData
{
public:
	unsigned int numParams;
	map<std::string, float> mData;
	map<int, std::string> keys;

	ParamData()
	{
		//
	}

	ParamData(unsigned int numParams, map<std::string, float> data)
	{
		this->numParams = numParams;
		mData = data;

		map<std::string, float>::iterator it;
		unsigned int i = 0;
		for(it = mData.begin(); i < numParams && it != mData.end(); it ++, i ++)
		{
			keys[i] = it->first;
		}
	}

	float operator[](std::string s)
	{
		if(mData.find(s) == mData.end())
		{
			return 0.0f;
		}
		return mData[s];
	}

	float operator[](int i)
	{
		return operator[](keys[i]);
	}
};

struct Camera
{
	Point2D<float> mCenter;
	float mZoomExp;
	float mScale;

	void offsetZoom(float s)
	{
		setZoom(mZoomExp + s);
	}

	void setZoom(float s)
	{
		mZoomExp = s;
		refreshZoom();
	}

	void refreshZoom()
	{
		mScale = pow(1.015f, mZoomExp);
	}
};

struct ColorSpectrum
{
	float spectrum;
	float minValue;
	float maxValue;

	ColorSpectrum()
	{
		spectrum = 1.0;
		minValue = 0.0;
	}

	void setSpectrum(float s)
	{
		if (s == 0)
		{
			s = 0.00001;
		}
		spectrum = s;
	}
};

class LocalizationUtil
{
public:
	static ParamData getDefaultParamData(XBox360RemoteControlI * controller)
	{
		map<std::string, float> theData;
		theData["ang_drift"] = 2.5;
		theData["trans_drift"] = 0.125;
		theData["num_particles"] = 750;
		theData["speed_scalar"] = 0.69125;
		theData["rotate_scalar"] = 1.5;
		theData["display_width"] = 1000;
		theData["display_height"] = 800;
		theData["cam_center_x"] = 0.0;
		theData["cam_center_y"] = 0.0;
		theData["cam_zoom"] = 150.0;
		theData["startingstate_x"] = -30.0;
		theData["startingstate_y"] = -30.0;
		theData["startingstate_angle"] = 90;
		theData["pool_dim_x"] = 75.0;
		theData["pool_dim_y"] = 75.0;

		if(controller)
		{
			theData["jsvals_l_x"] = controller->itsAxisIndices[XBox360RemoteControl::Keys::Actions::STRAFE]; //get the axis number currently mapped to this action
			theData["jsvals_l_y"] = controller->itsAxisIndices[XBox360RemoteControl::Keys::Actions::SPEED];
			theData["jsvals_r_x"] = controller->itsAxisIndices[XBox360RemoteControl::Keys::Actions::HEADING];
		}
		else
		{
			theData["jsvals_l_x"] = 0;
			theData["jsvals_l_y"] = 1;
			theData["jsvals_r_x"] = 3;
		}

		theData["jsvals_r_y"] = 4;
		theData["jsvals_trigger"] = 5;
		theData["jsvals_dpad_x"] = 6;
		theData["jsvals_dpad_y"] = 7;
		theData["bvals_l_bumper"] = 4;
		theData["bvals_r_bumper"] = 5;
		theData["bvals_rjs_click"] = 9;
		theData["bvals_y"] = 3;
		theData["bvals_b"] = 1;
		theData["bvals_x"] = 2;
		theData["bvals_a"] = 0;
		theData["bvals_start"] = 6;
		theData["bvals_back"] = 10;
		ParamData pd(31, theData);
		return pd;
	}

	static bool readConfigFile(string uri, ParamData &pd)
	{
		//////////[ Read in params ]//////////

		ifstream ifs(uri.c_str(), ifstream::in);
		cout << "Trying to read config file in " << uri << "..." << endl;
		if (!ifs.good())
		{
			cout << "Couldn't find config file." << endl;
			return false;
		}
		else
		{
			cout << "Config file opened..." << endl;
			map<std::string, float> theParams;
			int numParams = 0;

			bool go = ifs.good();
			int pos2 = 0;
			int pos3 = -1;

			while (go)
			{
				float theValue;
				string theTag;
				stringstream ss;
				string s;
				ifs >> s;
				if (ifs.good())
				{
					int pos1 = pos3 + 1;
					pos2 = s.find("=", pos1);
					pos3 = s.find("\n", pos2);
					theTag = s.substr(pos1, pos2 - pos1);
					pos2 ++;
					ss << s.substr(pos2, pos3 - pos2);
					ss >> theValue;
					theParams[theTag] = theValue;
					numParams ++;
				}
				else
				{
					go = false;
				}
			}

			ifs.close();

			ParamData theResult(numParams, theParams);
			pd = theResult;

			cout << "Config file read." << endl;
		}
		return true;

		////////////////////
	}

	static bool writeConfigFile(string uri, ParamData pd) //returns success/failure
	{
		cout << "Trying to write config file to " << uri << "..." << endl;
		ofstream ofs(uri.c_str());
		if (ofs.good())
		{
			cout << "Config file opened for writing..." << endl;
			map<std::string, float>::iterator it;
			for(it = pd.mData.begin(); it != pd.mData.end(); it ++)
			{
				ofs << it->first << "=" << it->second << endl;
			}
		}
		else
		{
			cout << "Couldn't write config file." << endl;
			return false;
		}
		ofs.close();
		cout << "Config file written." << endl;
		return true;
	}

	static float polarToEuler(float a) //degrees people. USE IT!
	{
		float result = a - 90; // conventional -> euler
		if(result < 0)
		{
			result += 360;
		}
		return result;
	}

	static float eulerToPolar(float a)
	{
		float result = a + 90;
		if(result < 0)
		{
			result += 360;
		}
		return result;
	}

	static float degToRad(float n)
	{
		return n * D_PI / 180;
	}

	static float radToDeg(float n)
	{
		return n * 180 / D_PI;
	}

	static float ProbabilityDistribution(float u, float s, float x)
	{
		return exp(-pow(x - u, 2) / (2 * pow(s, 2))) / sqrt(2 * D_PI
				* pow(s, 2));
	}

	static float floatMod(float f, float m)
	{
		//cout << "->floatMod" << endl;
		int a = (int) (f / m);

		if (a < 0)
		{
			f -= (float) (m * (a - 1));
		}
		else
		{
			f -= (float) (m * a);
		}

		//cout << "<-floatMod" << endl;
		return f;
	}

	static PixRGB<byte> RGBFromHSV(float H, float S, float V)
	{
		//cout << "->(" << H << "," << S << "," << V << ")RGBFromHSV" << endl;
		double R = 0, G = 0, B = 0;

		if (V == 0)
		{
			R = 0;
			G = 0;
			B = 0;
		}
		else if (S == 0)
		{
			R = V;
			G = R;
			B = R;
		}
		else
		{
			const double hf = H / 60.0;
			const int i = (int) floor(hf);
			const double f = hf - i;
			const double pv = V * (1 - S);
			const double qv = V * (1 - S * f);
			const double tv = V * (1 - S * (1 - f));
			switch (i)
			{
			case 0:
				R = V;
				G = tv;
				B = pv;
				break;
			case 1:
				R = qv;
				G = V;
				B = pv;
				break;
			case 2:
				R = pv;
				G = V;
				B = tv;
				break;
			case 3:
				R = pv;
				G = qv;
				B = V;
				break;
			case 4:
				R = tv;
				G = pv;
				B = V;
				break;
			case 5:
				R = V;
				G = pv;
				B = qv;
				break;
			case 6:
				R = V;
				G = tv;
				B = pv;
				break;
			case -1:
				R = V;
				G = pv;
				B = qv;
				break;
			default:
				break;
			}
		}

		R *= 255;
		G *= 255;
		B *= 255;

		//cout << "<-(" << R << "," << G << "," << B << ")RGBFromHSV" << endl;
		return PixRGB<byte> ((int) R, (int) G, (int) B);
	}

	static Point2D<float> vectorTo(Point2D<float> p1, Point2D<float> p2)
	{
		//cout << "->vectorTo" << endl;
		Point2D<float> p(p2.i - p1.i, p2.j - p1.j);
		float dist = sqrt(pow(p.i, 2) + pow(p.j, 2));
		float angle = 0.0;
		if (p.i == 0)
		{
			if (p.j >= 0)
			{
				angle = 90;
			}
			else
			{
				angle = 270;
			}
		}
		else
		{
			angle = abs(atan(p.j / p.i) / D_DEGREE);
			if (p.i >= 0 && p.j >= 0)
			{
				//
			}
			else if (p.i < 0 && p.j >= 0)
			{
				angle = (90 - angle) + 90;
			}
			else if (p.i < 0 && p.j < 0)
			{
				angle += 180;
			}
			else
			{
				angle = (90 - angle) + 270;
			}
		}
		angle = floatMod(angle, 360.0);
		//cout << "<-vectorTo" << endl;
		return Point2D<float> (dist, angle);
	}

	//calculates the ratio of difference between a1 and a2 such that angleRatio(x, x+180) = 1 and angleRatio(x, x) = 0
	static float linearAngleDiffRatio(float a1, float a2)
	{
		//cout << "->angleDiffRatio" << endl;
		float theAngle = abs(a1 - a2);
		while(theAngle > 360)
		{
			theAngle -= 360;
		}
		while(theAngle < 0)
		{
			theAngle += 360;
		}
		if(theAngle > 180)
		{
			theAngle = 360.0f - theAngle;
		}
		theAngle /= 180.0f;

		//cout << "<-angleDiffRatio" << endl;
		return theAngle;
	}

	static float trigAngleDiffRatio(float a1, float a2)
	{
		float d = sqrt(pow(cos(D_DEGREE * a1) - cos(D_DEGREE * a2), 2) + pow(sin(D_DEGREE * a1) - sin(D_DEGREE * a2), 2));
		return d / 2.0f;
	}
};

#endif /* LOCALIZATIONUTIL_H_ */
