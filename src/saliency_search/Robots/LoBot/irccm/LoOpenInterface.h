/**
   \file  Robots/LoBot/irccm/LoOpenInterface.h
   \brief Definitions for some of the Open Interface stuff used by the
   lobot low-level controller.
*/

/*
 ************************************************************************
 * The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   *
 * by the University of Southern California (USC) and the iLab at USC.  *
 * See http:  iLab.usc.edu for information about this project.          *
 *                                                                      *
 * Major portions of the iLab Neuromorphic Vision Toolkit are protected *
 * under the U.S. patent ``Computation of Intrinsic Perceptual Saliency *
 * in Visual Environments, and Applications'' by Christof Koch and      *
 * Laurent Itti, California Institute of Technology, 2001 (patent       *
 * pending; application number 09/912,225 filed July 23, 2001; see      *
 * http:  pair.uspto.gov/cgi-bin/final/home.pl for current status).     *
 ************************************************************************
 * This file is part of the iLab Neuromorphic Vision C++ Toolkit.       *
 *                                                                      *
 * The iLab Neuromorphic Vision C++ Toolkit is free software; you can   *
 * redistribute it and/or modify it under the terms of the GNU General  *
 * Public License as published by the Free Software Foundation; either  *
 * version 2 of the License, or (at your option) any later version.     *
 *                                                                      *
 * The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  *
 * that it will be useful, but WITHOUT ANY WARRANTY; without even the   *
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      *
 * PURPOSE.  See the GNU General Public License for more details.       *
 *                                                                      *
 * You should have received a copy of the GNU General Public License    *
 * along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   *
 * to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   *
 * Boston, MA 02111-1307 USA.                                           *
 ************************************************************************
*/

/*
   Primary maintainer for this file: Manu Viswanathan <mviswana at usc dot edu>
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoOpenInterface.h $
   $Id: LoOpenInterface.h 12717 2010-01-29 15:01:45Z mviswana $
*/

#ifndef LOBOT_IRCCM_OPEN_INTERFACE_DOT_H
#define LOBOT_IRCCM_OPEN_INTERFACE_DOT_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------- CONSTANTS -----------------------------*/

/// These enumerations are for the different Open Interface commands that
/// the iRobot Create understands.
enum {
   /// This command starts the Open Interface. The robot expects to be
   /// sent this command before it will start recognizing and responding
   /// to any other Open Interface commands.
   LOBOT_OI_CMD_START = 128,

   /// This command sets the baud rate at which Open Interface commands
   /// and data are sent. It expects a one-byte baud rate code parameter
   /// to be sent along with the command.
   ///
   /// The default baud rate on power up is 57,600 bits per second.
   /// However, if the user holds down the Play button while powering on
   /// the Create, the baud rate will be set to 19,200 bps.
   ///
   /// Once the baud rate is set/changed, it remains in effect until the
   /// robot is power-cycled or until its battery removed or until the
   /// battery voltage drops below the minimum required for processor
   /// operation.
   ///
   /// After sending this command, be sure to wait at least 100ms for the
   /// baud rate change to take effect and for subsequent commands and
   /// data to be sent at the new baud rate. Moreover, at the maximum
   /// supported baud rate of 115,200 bps, be sure to wait at least 200
   /// microseconds between consecutive send operations.
   ///
   /// DEVNOTE: This file defines symbolic constants for the supported
   /// baud rate codes and the corresponding values for the baud rate
   /// register UBRR0.
   LOBOT_OI_CMD_BAUD = 129,

   /// The Create has four operating modes, viz., Off, Passive, Safe and
   /// Full.
   ///
   /// In Passive mode, user programs can request sensor data and have
   /// the robot perfom any of its built-in demos, but cannot issue motor
   /// or actuator commands. The robot always powers up in Passive mode.
   ///
   /// In Safe mode, user programs have full control of the robot except
   /// when a cliff or wheel drop is detected. When that happens, the
   /// robot will immediately shut off its motors and revert to Passive
   /// mode. The same thing happens when the robot detects that its
   /// charger is plugged in.
   ///
   /// In Full mode, user programs have full control of the robot and it
   /// is up to user programs to detect and respond to cliff, wheel drop
   /// and charging events.
   //@{
   LOBOT_OI_CMD_PASSIVE = LOBOT_OI_CMD_START,
   LOBOT_OI_CMD_SAFE    = 131,
   LOBOT_OI_CMD_FULL    = 132,
   //@}

   /// This command can be used to run one of the built-in demo programs.
   /// It expects a one-byte parameter specifying the demo number.
   ///
   /// NOTE: This command will put the robot into Passive mode.
   ///
   /// DEVNOTE: This file defines symbolic constants for the demo
   /// numbers.
   LOBOT_OI_CMD_DEMO = 136,

   /// These commands can be used to run some of the demos directly
   /// (i.e., without having to use LOBOT_OI_CMD_DEMO + demo number).
   //@{
   LOBOT_OI_CMD_DEMO_SPOT  = 134,
   LOBOT_OI_CMD_DEMO_COVER = 135,
   LOBOT_OI_CMD_DEMO_COVER_AND_DOCK = 143,
   //@}

   /// This command controls the Create's drive wheels. It expects four
   /// data bytes. The first two data bytes specify the average speed of
   /// the drive wheels in mm/s. The next two data bytes specify the turn
   /// radius in mm.
   ///
   /// Both parameters are interpreted as signed 16-bit integers, high
   /// byte first. Positive values of the speed parameter result in the
   /// Create driving forwards while negative values result in it backing
   /// up. Positive values of the turn parameter result in left (ccw)
   /// turns and negative turn radii result in right (cw) turns. The turn
   /// radius is measured from the center of the robot to the center of
   /// the turning circle.
   ///
   /// The speed parameter must lie in the range [-500, 500] and the turn
   /// radius parameter in the range [-2000, 2000]. Larger turn radii
   /// result in the Create driving straighter whereas short turn radii
   /// cause the Create to execute sharp turns. Special values are used
   /// to indicate in-place turns and to drive straight ahead without
   /// turning.
   ///
   /// DEVNOTE: This file defines symbolic constants for the turn radius
   /// special cases.
   LOBOT_OI_CMD_DRIVE = 137,

   /// This command allows independent control of the two drive wheels.
   /// It expects four data bytes. The first two bytes are a signed
   /// 16-bit integer (high byte first) specifying the speed (mm/s) for
   /// the right wheel and the other two data bytes are the same thing
   /// for the left wheel.
   ///
   /// Positive values for a given parameter will make that wheel drive
   /// forwards, negative values will make it go backwards. Both speed
   /// parameters must lie in the range [-500, 500].
   LOBOT_OI_CMD_DRIVE_DIRECT = 145,

   /// This command controls the LEDs on the Create. It expects three
   /// data bytes. The first byte specifies the state of the Play and
   /// Advance LEDs. The second byte specifies the color of the Power LED
   /// and the third byte specifies the Power LED's intensity.
   ///
   /// Play and Advance LEDs: bit #1 is for the Play LED and bit #3 is
   /// for the Advance LED. If these bits are set to one, the respective
   /// LEDs will be on; if they are zero, the LEDs will be off. This file
   /// defines appropriate constants and masks for these LEDs.
   ///
   /// The Power LED: the second and third data bytes for this command
   /// are both unsigned numbers in the range [0, 255]. The second byte
   /// specifies the Power LED's color, which can glow green (0) to red
   /// (255). Intermediate values will result in colors such as orange,
   /// yellow, etc.
   ///
   /// The third byte controls the intensity with which the Power LED
   /// should glow. Zero means off, 255 means full intensity;
   /// intermediate values result in intermediate intensities.
   LOBOT_OI_CMD_LEDS = 139,

   /// This command defines a "song" for the robot to play at some later
   /// time. The robot supports up to 16 such songs. Each song can
   /// contain up to 16 notes. A note is specified using MIDI note
   /// definitions plus a duration specified in fractions of a second.
   ///
   /// This command expects a series of data bytes. The first byte must
   /// specify the song ID (0 to 15) and the second byte specifies the
   /// number of notes the song contains (1 to 16). Thereafter, come the
   /// notes in byte pairs (first the MIDI note number and the next the
   /// duration).
   LOBOT_OI_CMD_DEFINE_SONG = 140,

   /// This commands plays a song defined earlier with the SONG command.
   /// It expects one data byte, viz., the song number (0 to 15) to play.
   ///
   /// NOTE: The PLAY_SONG command won't work if another song is
   /// currently playing. The "song playing" sensor packet can be used to
   /// check whether the Create is ready to accept this command or not.
   LOBOT_OI_CMD_PLAY_SONG = 141,

   /// This command requests the robot to send a packet of sensor data
   /// bytes. There are 43 differenct snsor data packets. Each such
   /// packet provides the value(s) of a specific sensor or group of
   /// sensors.
   ///
   /// The commands expects one data byte, viz., the sensor packet ID.
   ///
   /// NOTE: Sensor queries should not be sent faster than 67Hz. That is,
   /// consecutive sensor queries should be at least 15ms apart.
   ///
   /// DEVNOTE: This file defines constants for the different sensor
   /// packets.
   LOBOT_OI_CMD_SENSORS = 142,

   /// This command requests a list of specific sensor packets instead of
   /// the predefined groups or single sensor values as in the SENSORS
   /// command. The robot returns the sensor packets in the order
   /// specified by the command.
   ///
   /// The command expects a series of data bytes. The first data byte
   /// must specify the number of sensor packets being requested. The
   /// remaining data bytes specify the desired packet IDs.
   ///
   /// NOTE: Sensor queries should not be sent faster than 67Hz. That is,
   /// consecutive sensor queries should be at least 15ms apart.
   ///
   /// DEVNOTE: This file defines constants for the different sensor
   /// packets.
   LOBOT_OI_CMD_QUERY_LIST = 149,
} ;

/// Enumerations for the supported baud rates. When the BAUD command is
/// issued, the following data byte should be one of these values.
enum {
   LOBOT_OI_BAUD_300,
   LOBOT_OI_BAUD_600,
   LOBOT_OI_BAUD_1200,
   LOBOT_OI_BAUD_2400,
   LOBOT_OI_BAUD_4800,
   LOBOT_OI_BAUD_9600,
   LOBOT_OI_BAUD_14400,
   LOBOT_OI_BAUD_19200,
   LOBOT_OI_BAUD_28800,
   LOBOT_OI_BAUD_38400,
   LOBOT_OI_BAUD_57600,
   LOBOT_OI_BAUD_115200,
} ;

/// Enumerations for the baud rate register values. When setting up the
/// Open Interface baud rate, the UBRR0 register will have to take on one
/// of these values.
enum {
   LOBOT_OI_UBRR_300    = 3839,
   LOBOT_OI_UBRR_600    = 1919,
   LOBOT_OI_UBRR_1200   = 959,
   LOBOT_OI_UBRR_2400   = 479,
   LOBOT_OI_UBRR_4800   = 239,
   LOBOT_OI_UBRR_9600   = 119,
   LOBOT_OI_UBRR_14400  = 79,
   LOBOT_OI_UBRR_19200  = 59,
   LOBOT_OI_UBRR_28800  = 39,
   LOBOT_OI_UBRR_38400  = 29,
   LOBOT_OI_UBRR_57600  = 19,
   LOBOT_OI_UBRR_115200 = 9,
} ;

/// These enumerations specify the demo program to be run when the
/// DEMO command is issued.
enum {
   /// Abort the demo that Create is currently performing.
   LOBOT_OI_DEMO_ABORT = 255,

   /// The Cover demo: Create attempts to cover an entire room using a
   /// comination of behaviours such as random bounce, wall following and
   /// spiraling.
   LOBOT_OI_DEMO_COVER = 0,

   /// The Cover and Dock demo: same as Cover except that if the Create
   /// sees an infrared signal from the Home Base, it docks and recharges
   /// itself.
   LOBOT_OI_DEMO_COVER_AND_DOCK,

   /// The Spot Cover demo: Create covers an around its starting position
   /// by spiraling outward and then inward.
   LOBOT_OI_DEMO_SPOT,

   /// The Mouse demo: Create locates a wall and then drives along it,
   /// traveling around the circumference of the room.
   LOBOT_OI_DEMO_MOUSE,

   /// The Figure Eight demo: Create continuously drives in a figure 8
   /// pattern.
   LOBOT_OI_DEMO_FIGURE_EIGHT,

   /// The Wimp demo: Create drives forward when pushed from behind; if
   /// it hits an obstacle, it drives away from the obstacle.
   LOBOT_OI_DEMO_WIMP,

   /// The Home demo: Create spins to locate a virtual wall and then
   /// drives toward it, stopping when it hits the wall or an obstacle.
   LOBOT_OI_DEMO_HOME,

   /// The Tag demo: Create spins to locate a virtual wall, then drives
   /// toward it. Upon reaching the virtual wall, it spins to find
   /// another virtual wall and drives toward it; so on and so forth,
   /// continuously bouncing from one virtual wall to the next.
   LOBOT_OI_DEMO_TAG,

   /// The Pachelbel demo: Create plays the notes of Pachelbel's Canon in
   /// sequence when cliff sensors are activated.
   LOBOT_OI_DEMO_PACHELBEL,

   /// The Banjo demo: Create plays a note for each of its four cliff
   /// sensors.
   LOBOT_OI_DEMO_BANJO,
} ;

/// Turn radius special cases.
enum {
   LOBOT_OI_DRIVE_STRAIGHT   = 0x8000,
   LOBOT_OI_DRIVE_STRAIGHT2  = 0x7FFF,
   LOBOT_OI_TURN_INPLACE_CW  = 0xFFFF,
   LOBOT_OI_TURN_INPLACE_CCW = 0x0001,
} ;

/// LED bit masks.
enum {
   LOBOT_OI_LED_ADVANCE = 0x08,
   LOBOT_OI_LED_PLAY    = 0x02,
   LOBOT_OI_LED_BOTH    = 0x0A,
} ;

/// Enumerations for the different sensor packet IDs.
enum {
   /// Sensor group zero returns packets 7 through 26.
   LOBOT_OI_SENSOR_GROUP_0,

   /// Sensor group one returns packets 7 through 16.
   LOBOT_OI_SENSOR_GROUP_1,

   /// Sensor group two returns packets 17 through 20.
   LOBOT_OI_SENSOR_GROUP_2,

   /// Sensor group three returns packets 21 through 26.
   LOBOT_OI_SENSOR_GROUP_3,

   /// Sensor group four returns packets 27 through 34.
   LOBOT_OI_SENSOR_GROUP_4,

   /// Sensor group five returns packets 35 through 42.
   LOBOT_OI_SENSOR_GROUP_5,

   /// Sensor group six returns all sensor packets, i.e., 7 through 42.
   LOBOT_OI_SENSOR_GROUP_6,

   /// Packet 7 is for the state of the bump and wheel drop sensors. A
   /// single byte is returned for these sensors. Bits 0 and 1 are for
   /// the right and left bump sensors respectively; bits 2, 3 and 4 are
   /// for the right, left and caster wheel drops respectively.
   ///
   /// A value of 1 for a bit means that that particular sensor is on;
   /// zero means the sensor is off.
   ///
   /// DEVNOTE: This file defines constants for the bit masks that can be
   /// used to examine this sensor packet's data/value byte.
   LOBOT_OI_SENSOR_BUMP_DROP,

   /// This packet returns a single byte specifying the state of the wall
   /// sensor. A value of 1 means that a wall has been detected; zero
   /// means no wall.
   LOBOT_OI_SENSOR_WALL,

   /// These packets return single bytes for the states of the cliff
   /// sensors. One implies a cliff; zero means no cliff.
   //@{
   LOBOT_OI_SENSOR_CLIFF_LEFT,
   LOBOT_OI_SENSOR_CLIFF_FRONT_LEFT,
   LOBOT_OI_SENSOR_CLIFF_FRONT_RIGHT,
   LOBOT_OI_SENSOR_CLIFF_RIGHT,
   //@}

   /// This packet returns a single byte specifying the state of the
   /// virtual wall sensor. A value of 1 means that a virtual wall has
   /// been detected; zero means no virtual wall.
   ///
   /// NOTE: The force field around the IR emitter atop the Home Base
   /// also trips this sensor.
   LOBOT_OI_SENSOR_VIRTUAL_WALL,

   /// This packet returns a single data byte specifying the value of the
   /// IR byte currently being received by the Create. These bytes can be
   /// sent by the Roomba Remote, the Home Base or by other Create robots
   /// and/or user-created devices.
   ///
   /// The value of this sensor is an unsigned byte in the range [0, 255].
   ///
   /// DEVNOTE: This file defines symbolic constants for the byte values
   /// that can be sent by the Roomba Remote and the Home Base.
   LOBOT_OI_SENSOR_INFRARED_BYTE = 17,

   /// This packet returns an unsigned byte indicating the state of the
   /// Create's Play and Advance buttons. Bit #0 specifies the state of
   /// the Play button and bit #2 is for the Advance button. When these
   /// bits are 1, the buttons are being pressed; zero means the buttons
   /// are not being pressed.
   ///
   /// DEVNOTE: This file defines symbolic constants that can be used as
   /// bit masks to ascertain the states of these two buttons using the
   /// data byte returned by this sensor packet.
   LOBOT_OI_SENSOR_BUTTONS,

   /// This packet returns the distance (in mm) the robot has travelled
   /// since the previous query of this sensor. The value is returned as
   /// a signed 16-bit integer (high byte first).
   ///
   /// NOTE: If this value is not polled often enough, it will be capped
   /// at its minimum or maximum. Also, if the wheels slip, the actual
   /// distance may be different from the one measured by the robot.
   LOBOT_OI_SENSOR_DISTANCE,

   /// This packet returns the angle (in degrees) that the robot has
   /// turned since the previous query of this sensor. The value is
   /// returned as a signed 16-bit integer (high byte first).
   ///
   /// NOTE: If this value is not polled often enough, it will be capped
   /// at its minimum or maximum. Also, if the wheels slip, the actual
   /// angle may be different from the one measured by the robot.
   LOBOT_OI_SENSOR_ANGLE,

   /// This packet returns the robot's current charging state. The return
   /// value is a single unsigned byte containing a code indicating the
   /// state.
   ///
   /// DEVNOTE: This file defines symbolic constants for the charging
   /// state.
   LOBOT_OI_SENSOR_CHARGING_STATE,

   /// This packet indicates the voltage (in mV) of the robot's battery.
   /// It is returned as an unsigned 16-bit number (high byte first).
   LOBOT_OI_SENSOR_VOLTAGE,

   /// This packet indicates the current (in mA) flowing into or out of
   /// the robot's battery. It is returned as a signed 16-bit number
   /// (high byte first). Negative values indicate current flowing out
   /// of the battery (as during normal operation) and positive values
   /// indicate current flowing into the battery (as during charging).
   LOBOT_OI_SENSOR_CURRENT,

   /// This packet returns a single signed data byte indicating the
   /// battery temperature (in degrees Celcius).
   LOBOT_OI_SENSOR_BATTERY_TEMP,

   /// This packet returns an unsigned 16-bit integer (high byte first)
   /// indicating the current charge in the robot's battery (in mAh). The
   /// charge value will decrease as the battery is depleted during
   /// normal operation and increase when it is being recharged.
   LOBOT_OI_SENSOR_BATTERY_CHARGE,

   /// This packet returns an unsigned 16-bit integer (high byte first)
   /// indicating the estimated charge capacity in the robot's battery
   /// (in mAh).
   LOBOT_OI_SENSOR_BATTERY_CAPACITY,

   /// This packet returns the strength of the wall sensor's signal in an
   /// unsigned 16-bit integer (high byte first) whose value can range
   /// from 0 to 4095.
   LOBOT_OI_SENSOR_WALL_SIGNAL,

   /// These packets return the strength of the cliff sensors' signals in
   /// unsigned 16-bit integers (high bytes first) whose values are in
   /// the range [0, 4095].
   //@{
   LOBOT_OI_SENSOR_CLIFF_LEFT_SIGNAL,
   LOBOT_OI_SENSOR_CLIFF_FRONT_LEFT_SIGNAL,
   LOBOT_OI_SENSOR_CLIFF_FRONT_RIGHT_SIGNAL,
   LOBOT_OI_SENSOR_CLIFF_RIGHT_SIGNAL,
   //@}

   /// This packet returns via an unsigned byte the availability of
   /// charging sources. If bit #0 is set, it means that the robot's
   /// internal charger is connected to a power supply; zero means this
   /// charging source is absent. If bit #1 is on, then the robot is
   /// connected to its Home Base; zero means not on the Home Base.
   ///
   /// DEVNOTE: This file defines appropriate bit mask constants to test
   /// this sensor packet's return value.
   LOBOT_OI_SENSOR_CHARGING_SOURCES = 34,

   /// This packet returns the current mode (Off, Passive, Safe or Full).
   /// The return value is a single unsigned byte.
   ///
   /// DEVNOTE: This file defines symbolic constants for the
   /// above-mentioned states.
   LOBOT_OI_SENSOR_MODE,

   /// This packet returns the currently selected song via a single
   /// unsigned data byte.
   LOBOT_OI_SENSOR_SONG_NUMBER,

   /// This packet returns one unsigned data byte, a flag indicating
   /// whether the robot is currently playing a song or not.
   LOBOT_OI_SENSOR_SONG_PLAYING,

   /// This packet returns the most recently requested drive command
   /// speed parameter via a signed 16-bit integer (high byte first). The
   /// return value will lie in the range [-500, 500].
   LOBOT_OI_SENSOR_REQUESTED_SPEED,

   /// This packet returns the most recently requested drive command turn
   /// radius parameter via a signed 16-bit integer (high byte first).
   /// The return value will lie in the range [-2000, 2000].
   LOBOT_OI_SENSOR_REQUESTED_RADIUS,

   /// These packets return the most recently requested drive direct
   /// command speed parameters via a signed 16-bit integer (high byte
   /// first). The return value will lie in the range [-500, 500].
   //@{
   LOBOT_OI_SENSOR_REQUESTED_SPEED_RIGHT,
   LOBOT_OI_SENSOR_REQUESTED_SPEED_LEFT,
   //@}
} ;

/// Sensor bit masks
enum {
   /// Bit masks to test the bump sensors.
   //@{
   LOBOT_OI_BUMP_LEFT   = 0x02,
   LOBOT_OI_BUMP_RIGHT  = 0x01,
   LOBOT_OI_BUMP_BOTH   = 0x03,
   LOBOT_OI_BUMP_EITHER = 0x03,
   //@}

   /// Bit masks to test the wheel drop sensors.
   //@{
   LOBOT_OI_WHEEL_DROP_CASTER = 0x10,
   LOBOT_OI_WHEEL_DROP_LEFT   = 0x08,
   LOBOT_OI_WHEEL_DROP_RIGHT  = 0x04,
   LOBOT_OI_WHEEL_DROP_ALL    = 0x1C,
   LOBOT_OI_WHEEL_DROP_ANY    = 0x1C,
   //@}

   /// Bit masks to test the states of the Play and Advance buttons.
   //@{
   LOBOT_OI_BUTTON_ADVANCE    = 0x04,
   LOBOT_OI_BUTTON_PLAY       = 0x01,
   //@}
} ;

/// Enumerations for the infrared bytes sent by the Roomba remote and the
/// Home base.
enum {
   LOBOT_OI_REMOTE_LEFT = 129,
   LOBOT_OI_REMOTE_FORWARD,
   LOBOT_OI_REMOTE_RIGHT,
   LOBOT_OI_REMOTE_SPOT,
   LOBOT_OI_REMOTE_MAX,
   LOBOT_OI_REMOTE_SMALL,
   LOBOT_OI_REMOTE_MEDIUM,
   LOBOT_OI_REMOTE_LARGE,
   LOBOT_OI_REMOTE_CLEAN = LOBOT_OI_REMOTE_LARGE,
   LOBOT_OI_REMOTE_PAUSE,
   LOBOT_OI_REMOTE_POWER,
   LOBOT_OI_REMOTE_ARC_FORWARD_LEFT,
   LOBOT_OI_REMOTE_ARC_FORWARD_RIGHT,
   LOBOT_OI_REMOTE_DRIVE_STOP,

   LOBOT_OI_HOME_BASE_FORCE_FIELD = 242,
   LOBOT_OI_HOME_BASE_GREEN_BUOY  = 244,
   LOBOT_OI_HOME_BASE_GBFF        = 246, // green buoy and force field
   LOBOT_OI_HOME_BASE_RED_BUOY    = 248,
   LOBOT_OI_HOME_BASE_RBFF        = 250, // red buoy and force field
   LOBOT_OI_HOME_BASE_RG_BUOY     = 252, // red and green buoys
   LOBOT_OI_HOME_BASE_RGBFF       = 254, // red and green buoys and force field
} ;

/// Enumerations for the current charging state.
enum {
   LOBOT_OI_CHARGING_NOT,
   LOBOT_OI_CHARGING_RECONDITIONING,
   LOBOT_OI_CHARGING_FULL,
   LOBOT_OI_CHARGING_TRICKLE,
   LOBOT_OI_CHARGING_WAITING,
   LOBOT_OI_CHARGING_FAULT,
} ;

/// Enumerations for the charging sources.
enum {
   LOBOT_OI_CHSRC_HOME_BASE = 0x02,
   LOBOT_OI_CHSRC_INTERNAL  = 0x01,
} ;

/// Enumerations for the robot's operational mode.
enum {
   LOBOT_OI_MODE_OFF,
   LOBOT_OI_MODE_PASSIVE,
   LOBOT_OI_MODE_SAFE,
   LOBOT_OI_MODE_FULL,
} ;

/// Enumerations for the number of bytes of sensor data returned for the
/// predefined sensor groups.
enum {
   LOBOT_OI_SENSOR_SIZE_GROUP_0 = 26,
   LOBOT_OI_SENSOR_SIZE_GROUP_1 = 10,
   LOBOT_OI_SENSOR_SIZE_GROUP_2 =  6,
   LOBOT_OI_SENSOR_SIZE_GROUP_3 = 10,
   LOBOT_OI_SENSOR_SIZE_GROUP_4 = 14,
   LOBOT_OI_SENSOR_SIZE_GROUP_5 = 12,
   LOBOT_OI_SENSOR_SIZE_GROUP_6 = 52,
} ;

/*---------------------------------------------------------------------*/

#ifdef __cplusplus
} // end of extern "C"
#endif

#endif
