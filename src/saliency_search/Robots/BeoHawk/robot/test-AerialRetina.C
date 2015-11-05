/*
 * test-AerialRetina.C
 */

#include <iostream>
#include <IceE/IceE.h>
#include <dc1394/dc1394.h>
#include "miniIce/ImageIce.h"
#include "miniIce/SimpleRobotSimEvents.h"
#include "SimpleRobotBrainComponent.H"

//NOTE IS CURRENTLY FOR NON-COMPRESSED 8-bit IMAGES
#define ICE_IMAGE_DEPTH                        1
#define ICE_FRAME_SKIP_RATE                4
#define EXPECTED_SIZE                        640*240
//the above is: in that number of frames, we should send 1,
// so basically it means that framerate across ice = 60/ICE_FRAME_SKIP_RATE


class AerialRetina : public SimpleRobotBrainComponent {
private:
        dc1394camera_t * camera;
        std::string cameraName;
        uint32_t cameraMode;
        int curSkip;

public:
        AerialRetina (dc1394camera_t * camera, std::string cameraName,
                        uint32_t cameraMode, char * iceStormIP, char * myName,
                        Ice::CommunicatorPtr ic) :
                        SimpleRobotBrainComponent(iceStormIP, myName, ic) {
                this->camera = camera;
                this->cameraName = cameraName;
                this->cameraMode = cameraMode;
                curSkip = 0;
        }
        virtual ~AerialRetina () {};

        void registerTopics() {
                this->registerPublisher("RetinaMessageTopic");
        }

        ImageIceMod::ImageIce dc2Ice(dc1394video_frame_t * input) {
                printf("here");
                ImageIceMod::ImageIce output;
                printf("here2");
                unsigned int height, width;
                dc1394_get_image_size_from_video_mode(camera, cameraMode, &width, &height);
                output.height = (int) height;
                output.width = (int) width;
                output.pixSize = ICE_IMAGE_DEPTH;

            int size = height*width*ICE_IMAGE_DEPTH;
            if (size == EXPECTED_SIZE*ICE_IMAGE_DEPTH) {
                    output.data.resize(size);
                    std::copy(input->image, input->image + size, output.data.begin());
            }

                return output;
        }

        void run () {
                registerTopics();
                this->start();

                if (dc1394_video_set_transmission(camera, DC1394_ON) != DC1394_SUCCESS) {
                        printf ("FATAL: Could not start camera stream.\n");
                        return;
                }
                dc1394video_frame_t * frame = NULL;

                while (true) {
                        RobotSimEvents::RetinaMessagePtr msg = new RobotSimEvents::RetinaMessage;
                        msg->cameraID = cameraName;

                        dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame);
                        printf ("gotframe...\n");
                        curSkip++;
                        if (curSkip < ICE_FRAME_SKIP_RATE) {
                                printf("about to send..\n");
                                msg->img = dc2Ice(frame);
                                this->publish("RetinaMessageTopic", msg);
                                printf("Sending image...\n");
                                curSkip = 0;
                        }

                        //for some reason the ENQUEUE?!?! function FREES a buffer
                        dc1394_capture_enqueue(camera, frame);
                }

                dc1394_video_set_transmission(camera, DC1394_OFF);
        }

};


int main (int argc, char* argv[]) {

        char * stormAddress = NULL;
        int cameraNum = -1;
        //STEP 1: PARSE CMDLINE ARGS
        // we don't have that nice model* stuffs, so we have to do this the very hard,
        // very old fashion way, sorry :(
        for (argc--, argv++; argc > 0; argc--, argv++) {
                if (strstr(*argv, "--icestorm-ip") != NULL)
                        stormAddress = *argv + strlen("--icestorm-ip=");
                else if (strstr(*argv, "--camera-num") != NULL)
                        cameraNum = atoi(*argv + strlen("--camera-num="));
        }

        if (stormAddress == NULL || cameraNum == -1) {
                printf("usage : test-AerialRetina --icestorm-ip=ICESTORM_IP_ADDRESS --camera-num=#\n");
                return 1;
        }

        //=================================================
        //STEP 2: initialize dc1394 stuffs:
        //=================================================
    dc1394camera_t * camera = NULL;
    dc1394_t * d;
    dc1394camera_list_t * list;
    uint32_t cameraMode;

        printf ("Initializing cameras...");
    d = dc1394_new ();

    if (d && dc1394_camera_enumerate (d, &list) == DC1394_SUCCESS &&
                    list->num > 0 && cameraNum > -1 && cameraNum < list->num) {

                camera = dc1394_camera_new (d, list->ids[cameraNum].guid);
                if (!camera) {
                        printf ("FAILURE - getting first camera\n");
                    return 1;
                }

                //clean camera:
                dc1394_reset_bus(camera);

                //setup the camera:
                dc1394video_modes_t modes;
                dc1394error_t err;
            dc1394_video_get_supported_modes(camera, &modes);
            cameraMode = modes.modes[modes.num-1];
            dc1394_video_set_iso_speed(camera, DC1394_ISO_SPEED_400);
            dc1394_video_set_mode(camera, modes.modes[modes.num-1]);
            err=dc1394_video_set_framerate(camera, DC1394_FRAMERATE_60);
            DC1394_ERR_RTN(err, "Could not set framerate\n");
            err=dc1394_capture_setup(camera, 4, DC1394_CAPTURE_FLAGS_DEFAULT);
            DC1394_ERR_RTN(err, "Could not setup camera-\nmake sure that the video mode and framerate are\nsupported by your camera\n");

    } else {
            printf ("FAILURE - initialization & enumeration\n");
            return 1;
    }

    dc1394_camera_free_list (list);
        printf ("SUCCESS\n");


        //==========================================
        //STEP 3: Initialize ICE stuffs:
        //==========================================

        printf ("Initializing ICE Components...");
        char nameBuf[64];
        sprintf(nameBuf, "camera%i", cameraNum);
        std::string cameraName(nameBuf);
        AerialRetina ar(camera, cameraName, cameraMode, stormAddress,
                        "AerialRetina", Ice::initialize(argc, argv));
        ar.registerTopics();
        printf ("SUCCESS\n");

        ar.run();
        //Yay, running:
        // d^.^b

        //Clean up dc1394 stuffs:
        dc1394_camera_free(camera);
        dc1394_free(d);

        return 0;
}
