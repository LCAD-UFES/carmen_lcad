#include <StampedGPSPose.hpp>

#include <carmen/global.h>
#include <carmen/Utm_To_Gdc_Converter.h>
#include <carmen/Gdc_To_Utm_Converter.h>

#include <carmen/Utm_Coord_3d.h>
#include <carmen/Gdc_Coord_3d.h>

#include <iostream>

using namespace hyper;

// basic constructor
StampedGPSPose::StampedGPSPose(unsigned msg_id) : StampedMessage(msg_id), gps_measurement(0.0, 0.0, M_PI), gps_std(0.0) {}

// basic destructor
StampedGPSPose::~StampedGPSPose() {}

// parse the pose from string stream
bool StampedGPSPose::FromCarmenLog(std::stringstream &ss)
{
    // helpers
    double lt_dm, lt, lg_dm, lg, sl;
    char lt_orientation, lg_orientation;
    int quality;

    std::string gid;

    // get the gps id
    ss >> gid;
    gid = "_" + gid;

    if (gps_id == gid)
    {
        // discards the second value
        SkipValues(ss, 1);

        // read the latitude in the dm format
        ss >> lt_dm;

        // read the latitude orientation
        ss >> lt_orientation;

        // read the longitude in the dm format
        ss >> lg_dm;

        // read the longitude orientation
        ss >> lg_orientation;

        // read the gps quality
        ss >> quality;

        // discards the next two values
        SkipValues(ss, 2);

        // get the sea level
        ss >> sl;

        // discards the next 4 values
        SkipValues(ss, 4);

        // get the timestamp
        ss >> StampedMessage::timestamp;

        // @Filipe: desvios padrao para cada modo do GPS Trimble.
        // @Filipe: OBS: Multipliquei os stds por 2 no switch abaixo para dar uma folga.
        // 0: DBL_MAX
        // 1: 4.0
        // 2: 1.0
        // 4: 0.1
        // 5: 0.1

        switch (quality)
        {
            case 1:
                gps_std = 16.0;
                break;
            case 2:
                gps_std = 4.0;
                break;
            case 4:
                gps_std = 0.4;
                break;
            case 5:
                gps_std = 0.1;
                break;
            default:
                gps_std = std::numeric_limits<double>::max();
                break;
        }

        // convert the latitude format
        lt = carmen_global_convert_degmin_to_double(lt_dm);

        // convert the longitude format
        lg = carmen_global_convert_degmin_to_double(lg_dm);

        // verify the latitude and longitude orientations
        if ('S' == lt_orientation) lt = -lt;
        if ('W' == lg_orientation) lg = -lg;

        // convert to x and y coordinates
        Gdc_Coord_3d gdc = Gdc_Coord_3d(lt, lg, sl);

        // Transformando o z utilizando como altitude a altitude mesmo - que esta vindo como zero
        Utm_Coord_3d utm;
        Gdc_To_Utm_Converter::Init();
        Gdc_To_Utm_Converter::Convert(gdc , utm);

        // update the measure without orientation
        gps_measurement.setTranslation(Eigen::Vector2d(utm.y, -utm.x));

        gps_measurement = gps_measurement;

        // set the estimate
        StampedMessage::est = gps_measurement;

        // the orientation values stays the same for now
        return true;
    }

    return false;
}

// get the stamped message type
StampedMessageType StampedGPSPose::GetType()
{
    return StampedGPSMessage;
}

// the external identifier
std::string StampedGPSPose::gps_id = "_1";
g2o::SE2 StampedGPSPose::inv_gps_pose = g2o::SE2(0.0, 0.0, 0.0);