/**
   \file  Robots/LoBot/irccm/LoCMInterface.h
   \brief Header file defining the interface between the low- and
   high-level parts of the Robolocust controller.

   The Robolocust controller consists of two major parts:

      1. The high-level C++ layers that run different behaviours, LGMD
         models, integration algorithms, etc.

      2. The low-level layer that runs on the microcontroller that
         interfaces with the electromechanical systems on the physical
         robot platform.

   The two parts communicate with each other by exchanging messages
   (commands and data) over a serial port. This file defines the
   interface between the higher layers and the low-level control program
   when the "roomba_cm" robot platform is being used. This platform
   identifies an iRobot Create with a Command Module in place.

   This header file specifies the symbolic IDs and the values of the
   different messages that are to go back and forth between the
   high-level C++ part of lobot and the low-level Command Module part. It
   also defines other relevant things such as the size of command
   messages sent by the high-level to the low-level and the number of
   bytes of sensor data sent from the low-level to the high-level.

   This file will have to be included by the high-level C++ codebase as
   well as the Command Module's low-level C codebase.
*/

/*
 ************************************************************************
 * The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   *
 * by the University of Southern California (USC) and the iLab at USC.  *
 * See http://iLab.usc.edu for information about this project.          *
 *                                                                      *
 * Major portions of the iLab Neuromorphic Vision Toolkit are protected *
 * under the U.S. patent ``Computation of Intrinsic Perceptual Saliency *
 * in Visual Environments, and Applications'' by Christof Koch and      *
 * Laurent Itti, California Institute of Technology, 2001 (patent       *
 * pending; application number 09/912,225 filed July 23, 2001; see      *
 * http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     *
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
   Primary maintainer for this file: mviswana usc edu
   $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/irccm/LoCMInterface.h $
   $Id: LoCMInterface.h 13782 2010-08-12 18:21:14Z mviswana $
*/

#ifndef LOBOT_IRCCM_COMMAND_MODULE_INTERFACE_DOT_H
#define LOBOT_IRCCM_COMMAND_MODULE_INTERFACE_DOT_H

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------- CONSTANTS -----------------------------*/

/// These enumerations are for acknowledgement (ACK) messages sent from
/// the low-level control program to the higher (C++) layers of the lobot
/// controller.
///
/// DEVNOTE: We use printable ASCII characters as the command codes to
/// ensure that we can easily identify the messages going back and forth
/// (when these message IDs are printed out during debugging).
enum {
   /// The Command Module sends the READY message to the high level to
   /// let it know that the low-level control program is ready to accept
   /// high-level commands. The high level must wait to receive this
   /// acknowledgement from the low level before sending a motor or
   /// sensor query command. Once the low level has executed the
   /// high-level command, it will again send a READY message to indicate
   /// that it is ready to accept the next command; so on and so forth.
   ///
   /// The high level cannot send commands willy-nilly and must wait for
   /// READY ACKs because the Command Module has only one USART that can
   /// either talk to the (external) USB port or the (internal) serial
   /// port to which the robot is connected. If the high level were to
   /// send commands at its whim, they may never be received by the
   /// Command Module because it may well have redirected its USART to
   /// the internal serial port and not be listening to whatever the high
   /// level has to say. Then, when the Command Module redirects its
   /// USART to the USB port, it may not receive any commands because the
   /// high level has fallen silent in the mean time.
   ///
   /// Needless to say, the above situation will, very quickly, make the
   /// robot quite schizophrenic. Therefore, the high and low level
   /// controllers need to coordinate their activities so as to elicit
   /// coherent behaviour on the part of the robot.
   ///
   /// When the low level sends ACK_READY, the high level can be assured
   /// that its command, if sent within a predefined timeout window
   /// (usually, 1.5 seconds), will be received and executed by the
   /// robot.
   LOBOT_ACK_READY = ':',

   /// The low level sends the BUMPS acknowledgement after reacting to
   /// the robot's bump sensors.
   ///
   /// The low level always reacts to the bump sensors by backing up a
   /// little and then spinning around a little bit. In case one or both
   /// of the rear bumpers is hit, the low level will respond by moving
   /// forward a little and the spinning. Thus, this message is
   /// accompanied by five bytes of data.
   ///
   /// The first byte indicates the sensor flags. Bit #0 is for the right
   /// bump sensor and bit #1 for the left bump sensor. The bit masks
   /// defined in LoOpenInterface.h may be used to test these bits. Bit
   /// #2 is for the rear right bumper and bit #3 for the rear left
   /// bumper. This file defines appropriate constants that can be used
   /// to test for the states of the rear bump sensors.
   ///
   /// The second and third data bytes make up a signed 16-bit integer
   /// (high byte first) indicating the distance backed up. Since the
   /// robot always backs up in response to a bump sensor, this number
   /// will always be negative.
   ///
   /// The fourth and fifth data bytes make up a signed 16-bit integer
   /// (high byte first) indicating the number of degrees turned in-place
   /// after the back-up. Positive values indicate counterclockwise turns
   /// and negative values are for clockwise turns.
   ///
   /// DEVNOTE: As mentioned above, the ACK_BUMPS message is followed by
   /// several data bytes containing bump sensor information as well as
   /// information about the actions the robot took in response to the
   /// bump sensors. This file defines a symbolic constant specifying how
   /// many such bytes the high level may expect to receive. Furthermore,
   /// this file also defines symbolic constants that can be used as
   /// indices into the data byte array sent as part of the
   /// acknowledgement.
   LOBOT_ACK_BUMPS = 'B',

   /// When the robot's wheel drop sensors are activated, the low-level
   /// controller will immediately shut off the motors and then send this
   /// message to the high level to let it know what has happened.
   ///
   /// This acknowledgement is accompanied by one data byte containing
   /// bit flags indicating which wheel drops are active. Bit #2 is for
   /// the right wheel, bit #3 for the left wheel and bit #4 for the
   /// front caster wheel. LoOpenInterface.h defines bit masks that can
   /// be used to test these bits.
   LOBOT_ACK_WHEEL_DROPS = 'W',

   /// When the robot's cliff sensors fire, the low-level controller
   /// backs up and then spins the robot to avoid falling down the stairs
   /// or something like that. To let the high level know what just
   /// happened, it sends this acknowledgement message accompanied by five
   /// data bytes.
   ///
   /// The first data byte contains the flags for the cliff sensors. Bit
   /// #3 of this data byte specifies whether the left cliff sensor is on
   /// or off; bit #2 is for the front left cliff sensor; bit #1 for
   /// front right; and bit #0 for the right cliff sensor.
   ///
   /// The second and third data bytes make up a signed 16-bit integer
   /// (high byte first) indicating the distance backed up. Since the
   /// robot always backs up in response to a cliff sensor, this number
   /// will always be negative.
   ///
   /// The fourth and fifth data bytes make up a signed 16-bit integer
   /// (high byte first) indicating the number of degrees turned in-place
   /// after the back-up. Positive values indicate counterclockwise turns
   /// and negative values are for clockwise turns.
   ///
   /// DEVNOTE: As mentioned above, the ACK_BUMPS message is followed by
   /// several data bytes containing bump sensor information as well as
   /// information about the actions the robot took in response to the
   /// bump sensors. This file defines a symbolic constant specifying how
   /// many such bytes the high level may expect to receive. Furthermore,
   /// this file also defines symbolic constants that can be used as
   /// indices into the data byte array sent as part of the
   /// acknowledgement. However, there are no symbolic bit mask constants
   /// for testing the bit flags in the first data byte.
   LOBOT_ACK_CLIFFS = 'C',

   /// When the robot is being remote controlled by the user, the low
   /// level will handle the requested command and then inform the high
   /// level with an ACK_REMOTE.
   ///
   /// This message is accompanied by a single data byte, which simply
   /// specifies the remote control command the low level handled.
   /// LoOpenInterface.h defines symbolic constants for the Roomba
   /// Remote's infrared byte.
   LOBOT_ACK_REMOTE = '.',

   /// The low level regularly polls the iRobot Create for all its sensor
   /// data and streams this data to the high level using the SENSORS
   /// acknowledgement. This message is followed by several bytes of
   /// sensor data sent from the low level to the high level. It is the
   /// high level controller's responsibility to buffer the sensor data
   /// being streamed by the low-level.
   ///
   /// DEVNOTE: As mentioned above, the ACK_SENSORS message is followed
   /// by several data bytes containing the current sensor values. This
   /// file defines a symbolic constant specifying how many such bytes
   /// the high level may expect to receive. Furthermore, this file also
   /// defines symbolic constants that can be used as indices into the
   /// sensor data byte array.
   LOBOT_ACK_SENSORS = '$',
} ;

/// The following enumerations are for command (CMD) messages sent from
/// the high-level controller to the low-level control program running on
/// the Command Module. These commands indicate the actions the high
/// level would like the robot to perform. The low-level control program
/// will respond to these commands by talking in Open Interface to the
/// iRobot Create. Thus, the high level need not worry about the
/// low-level minutiae required to get the robot to actually execute such
/// actions as "drive left at 200mm/s" or "veer right while backing up at
/// 100mm/s," etc.
///
/// NOTE: All commands sent by the high level must consist of exactly
/// four bytes. The first byte holds the command ID (one of the CMD enums
/// defined below), the second and third bytes are for a 16-bit parameter
/// (e.g., the drive speed or turn radius), and the fourth byte must be a
/// parity checksum byte computed by taking the bitwise XOR of the other
/// three bytes.
///
/// The low level will execute the command only if the parity byte checks
/// out okay. If not, the low-level controller will stop the robot. It
/// will also stop the robot if it gets an unknown command ID.
///
/// As a matter of good form, the high level should set the two parameter
/// bytes to zero if the command doesn't require a parameter (e.g.,
/// CMD_STOP). If a command does require a parameter, it should be sent
/// high byte first followed by the low byte.
///
/// DEVNOTE: We use printable ASCII characters as the command codes to
/// ensure that we can easily identify the messages going back and forth
/// (when these message IDs are printed out during debugging).
enum {
   /// Since the high- and low-level controllers operate asynchronously,
   /// it is possible that when the low level sends ACK_READY, the high
   /// level doesn't have any real drive command to perform. In that
   /// case, it could decide not to send any command to the low level.
   ///
   /// Unfortunately, the low level policy is to stop the robot if it
   /// doesn't get a valid command within its predefined timeout window.
   /// Thus, not sending a command is probably not the best thing to do.
   /// Instead, the high level should send a NOP command to ensure that
   /// the robot continues to do whatever it is currently doing.
   ///
   /// NOTE: Although not recommended, the high level may send a long
   /// string of NOPs since the low level responds to the robot's bump,
   /// wheel drop, cliff and wall sensors to ensure that the robot
   /// doesn't drive off a cliff or keep its motors running even when it
   /// is butting heads against a wall. Whenever the low-level controller
   /// takes some action in response to the robot's built-in/low-level
   /// sensors, it sends a corresponding acknowledgement to the high
   /// level to let the high level know what has happened.
   ///
   /// Nonetheless, as mentioned above, the NOP command is really meant
   /// for occasional use when the high-level command queue is either
   /// freshly empty (i.e., all pending commands just got sent out and
   /// the low level responded with an ACK_READY sooner than expected) or
   /// empty because no behaviour has gotten around to voting for an
   /// action just yet or because no behaviour has sensed anything
   /// alarming that needs an immediate response.
   LOBOT_CMD_NOP = '@',

   /// These commands drive the robot. CMD_FORWARD and CMD_REVERSE
   /// require a parameter: a positive number specifying the drive speed
   /// in mm/s. This number must lie in the range zero to 500. A value of
   /// zero is equivalent to CMD_STOP. Negative numbers will be treated
   /// as zero. Numbers greater than 500 will be clamped to 500.
   ///
   /// CMD_STOP does not need a parameter.
   //@{
   LOBOT_CMD_FORWARD = 'F',
   LOBOT_CMD_REVERSE = 'R',
   LOBOT_CMD_STOP    = 'S',
   //@}

   /// These commands steer the robot. CMD_LEFT and CMD_RIGHT require a
   /// parameter: a positive number in the range [0, 2000] specifying the
   /// turn radius in mm. A value of zero will result in an in-place
   /// turn.
   ///
   /// CMD_STRAIGHT does not need a parameter.
   //@{
   LOBOT_CMD_LEFT     = '<',
   LOBOT_CMD_RIGHT    = '>',
   LOBOT_CMD_STRAIGHT = '!',
   LOBOT_CMD_SPIN     = '*',
   //@}

   /// The robot sports a pair of Sharp GP2D15 IR proximity detectors on
   /// its rear to help prevent bumping into objects as it reverses.
   /// Unfortunately, these detectors also fire when the user's hand
   /// mucks about with the Command Module's on/off switch or when the
   /// robot is picked up and placed at some specific initial position.
   /// This behaviour can be very annoying because we might require the
   /// robot to remain stationary until the commencement of an
   /// experiment/run.
   ///
   /// To work around the above issue, the low-level controller supports
   /// these commands for enabling and disabling the rear bumpers. Thus,
   /// the high-level controller can dictate exactly when it is alright
   /// for the rear bumpers to be active.
   ///
   /// Additionally, the low-level controller can respond to rear bump
   /// events by spinning the robot in-place by a little bit and then
   /// moving it forward a small amount. The default response, however,
   /// is to simply stop the robot when either of the rear bumpers is
   /// active. To elicit the spin-and-move action, the high-level
   /// controller should send a '1' as the parameter for the
   /// LOBOT_CMD_ENABLE_REAR_BUMPS command. A value of zero for this
   /// command's parameter will result in the robot only stopping when
   /// one of the rear bump sensors is active.
   //@{
   LOBOT_CMD_ENABLE_REAR_BUMPS  = 'e',
   LOBOT_CMD_DISABLE_REAR_BUMPS = 'd',
   //@}
} ;

/// These enumerations specify the sizes of different message or data
/// packets exchanged between the high- and low-level controllers.
enum {
   /// This symbolic constant specifies the number of bytes that each
   /// command sent by the high level to the low level must be.
   ///
   /// As described earlier, all high-level commands are expected to be
   /// four bytes long. The first byte holds the command ID, the next two
   /// bytes are for a 16-bit parameter (sent high byte first) and the
   /// fourth byte is an XOR parity byte.
   ///
   /// DEVNOTE: If the format of a high-level command ever changes, be
   /// sure to update this number to follow suit. Otherwise, expect Bad
   /// Things to happen.
   LOBOT_CMD_SIZE = 4,

   /// The number of data bytes sent back with the ACK_BUMPS message.
   ///
   /// DEVNOTE: If the format and/or size of the ACK_BUMPS message ever
   /// changes, be sure to update this number to follow suit. Otherwise,
   /// expect Bad Things to happen.
   LOBOT_BUMPS_SIZE = 5,

   /// The number of data bytes sent back with the ACK_WHEEL_DROPS
   /// message.
   ///
   /// Only the state of the wheel drops as returned by the Open
   /// Interface is sent as part of this message. Thus, there is only one
   /// data byte. It contains 3 bits indicating the status of the wheel
   /// drops. Bit #2 is for the right wheel, bit #3 for the left wheel,
   /// and bit #4 for the front caster wheel. LoOpenInterface.h defines
   /// bit masks for testing these bit flags.
   ///
   /// DEVNOTE: If the format and/or size of the ACK_WHEEL_DROPS message
   /// ever changes, be sure to update this number to follow suit.
   /// Otherwise, expect Bad Things to happen.
   LOBOT_WHEEL_DROPS_SIZE = 1,

   /// The number of data bytes sent back with the ACK_CLIFFS message.
   ///
   /// DEVNOTE: If the format and/or size of the ACK_CLIFFS message ever
   /// changes, be sure to update this number to follow suit. Otherwise,
   /// expect Bad Things to happen.
   LOBOT_CLIFFS_SIZE = 5,

   /// The number of data bytes sent back with the ACK_REMOTE message.
   ///
   /// Only the infrared byte received and processed by the low level is
   /// sent as part of this message. LoOpenInterface.h defines symbolic
   /// constants for the IR byte's different values.
   ///
   /// DEVNOTE: If the format and/or size of the ACK_REMOTE message ever
   /// changes, be sure to update this number to follow suit. Otherwise,
   /// expect Bad Things to happen.
   LOBOT_REMOTE_SIZE = 1,

   /// This symbol specifies the number of data bytes that will be sent
   /// back to the high level along with the ACK_SENSORS acknowledgement
   /// message.
   ///
   /// DEVNOTE: At present, we send back all of the Create's sensor
   /// packets plus one extra byte to the high level and let it decide
   /// what data it wants/needs and what it doesn't want/need. Therefore,
   /// the number of data bytes sent back will be one more than the size
   /// of the sensor data buffer for group 6 sensor packets as defined in
   /// the Open Interface.
   ///
   /// We could simply use the LOBOT_OI_SENSOR_SIZE_GROUP_6 symbol
   /// defined in irccm/LoOpenInterface.h over here. However, we choose
   /// to "hard-code" the number 53 (i.e., 52 + 1) instead. This allows
   /// us to avoid having to include LoOpenInterface.h in this file, a
   /// consideration that seems to make sense because LoOpenInterface.h
   /// defines things that are pertinent to the Open Interface whereas
   /// this header defines things specifically pertinent to the lobot
   /// controller.
   ///
   /// The fact that LoOpenInterface.h defines a symbol that happens to
   /// match the number of sensor data bytes the low-level lobot
   /// controller sends to the high level is somewhat (though not
   /// entirely) coincidental. In the future, we may decide to send a
   /// subset of the available sensor packets or to preprocess/combine
   /// the sensor data returned by the robot before sending it off to
   /// the high level. In that case, LoOpenInterface.h may not define a
   /// convenient symbol specifying the number of data bytes sent to the
   /// high-level lobot controller. This is why it makes sense to
   /// decouple this header file from LoOpenInterface.h.
   ///
   /// DEVNOTE 2: If, as alluded to above, we do decide to change the
   /// sensor data that is to be sent back to the high level, be sure to
   /// update this number here! Otherwise, Bad Things *will* happen.
   LOBOT_SENSORS_SIZE = 53,
} ;

/// These enumerations specify the indices for the data byte array sent
/// from the low level to the high level as part of the ACK_BUMPS
/// message.
enum {
   /// The first data byte indicates the sensor flags. Bit #0 is for the
   /// right bump sensor and bit #1 for the left bump sensor. The bit
   /// masks defined in LoOpenInterface.h may be used to test these bits.
   ///
   /// Additionally, lobot sports two rear bump sensors. Their bit flags
   /// are in bits 3 (rear right) and 4 (rear left). Since these bump
   /// sensors are not built into the iRobot Create, they do not have
   /// predefined masks as part of the Open Interface. Instead, this file
   /// defines appropriate masks that can be used to test the status of
   /// the rear bumpers.
   LOBOT_BUMPS_FLAGS,

   /// The second and third data bytes make up a signed 16-bit integer
   /// (high byte first) indicating the distance backed up or moved
   /// forward in response to the front or rear bumpers.
   LOBOT_BUMPS_DISTANCE_HI,
   LOBOT_BUMPS_DISTANCE_LO,

   /// The fourth and fifth data bytes make up a signed 16-bit integer
   /// (high byte first) indicating the number of degrees turned in-place
   /// as part of the bump sensor response. Positive values indicate
   /// counterclockwise turns and negative values are for clockwise
   /// turns.
   LOBOT_BUMPS_ANGLE_HI,
   LOBOT_BUMPS_ANGLE_LO,
} ;

/// These enumerations specify the indices for the data byte array sent
/// from the low level to the high level as part of the ACK_CLIFFS
/// message.
enum {
   /// The first data byte indicates the cliff sensor flags. Bit #0 is
   /// for the right cliff sensor; bit #1 for the front right cliff
   /// sensor; bit #2 for the front left cliff sensor; and bit #3 for the
   /// left cliff sensor.
   LOBOT_CLIFFS_FLAGS,

   /// The second and third data bytes make up a signed 16-bit integer
   /// (high byte first) indicating the distance backed up. Since the
   /// robot always backs up in response to a cliff sensor, this number
   /// will always be negative.
   LOBOT_CLIFFS_DISTANCE_HI,
   LOBOT_CLIFFS_DISTANCE_LO,

   /// The fourth and fifth data bytes make up a signed 16-bit integer
   /// (high byte first) indicating the number of degrees turned in-place
   /// after the back-up. Positive values indicate counterclockwise turns
   /// and negative values are for clockwise turns.
   LOBOT_CLIFFS_ANGLE_HI,
   LOBOT_CLIFFS_ANGLE_LO,
} ;

/// These enumerations specify the indices for the sensor data byte array
/// sent from the low level to the high level as part of the ACK_SENSORS
/// message.
///
/// DEVNOTE: At present, we simply send back all of the Create's sensor
/// packets to the high level plus one extra byte. Thus, the order of
/// sensor data bytes matches exactly the order specified in the Open
/// Interface specs for sensor packet #6. The last byte, however, is the
/// extra byte alluded to above; it is not part of the Open Interface
/// specs.
enum {
   /// The first byte of the sensor data contains the bits for the bump
   /// sensors and the wheel drop sensors. Bits 0 and 1 are for the right
   /// and left bump sensors respectively. Bits 2, 3 and 4 are for the
   /// right, left and caster wheel drops respectively. Bits 5 and 6 are
   /// for the right and left bump sensors added to the rear of the
   /// robot.
   ///
   /// DEVNOTE: LoOpenInterface.h defines symbolic constants for bit
   /// masks that can be used to access these individual bits. This file
   /// defines bit masks for bits 5 and 6, which are for the added rear
   /// bumpers.
   LOBOT_SENSORS_BUMPS = 0,
   LOBOT_SENSORS_WHEEL_DROPS = LOBOT_SENSORS_BUMPS,

   /// The second byte of sensor data contains a flag indicating whether
   /// the Create/Roomba's wall sensor is active (1) or not (0).
   LOBOT_SENSORS_WALL,

   /// Bytes 3 through 6 are flags indicating whether the cliff sensors
   /// are active (1) or not (0).
   //@{
   LOBOT_SENSORS_CLIFF_LEFT,
   LOBOT_SENSORS_CLIFF_FRONT_LEFT,
   LOBOT_SENSORS_CLIFF_FRONT_RIGHT,
   LOBOT_SENSORS_CLIFF_RIGHT,
   //@}

   /// The seventh byte of sensor data returned by the low level specifies
   /// whether the virtual wall sensor is active (1) or not (0).
   LOBOT_SENSORS_VIRTUAL_WALL,

   /// Byte #8 specifies the state of the low side driver and wheel
   /// overcurrents.
   ///
   /// DEVNOTE: As of now, LoOpenInterface.h does not provide any
   /// convenient means of making sense of this particular value. Refer
   /// to the Open Interface specs for the gory details.
   LOBOT_SENSORS_LSD_WHEEL_OC,

   /// After the wheel overcurrents, come two unused bytes (always
   /// zero).
   LOBOT_UNUSED1,
   LOBOT_UNUSED2,

   /// The 11th byte contains the IR command received by the robot's
   /// omnidirectional IR sensor located in the front of the robot on top
   /// of the bumper assembly. This sensor value lies in the range [0, 255].
   ///
   /// DEVNOTE: LoOpenInterface.h defines symbolic constants for the
   /// different values sent by the Roomba Remote and the Home Base.
   LOBOT_SENSORS_INFRARED_BYTE,

   /// The 12th byte specifies the states of the Play and Advance
   /// buttons. Bit #0 is for the Play button and bit #2 is for the
   /// Advance button. If these bits are 1, it means the corresponding
   /// button is currently being pressed; zero means the button is not
   /// being pressed.
   ///
   /// DEVNOTE: LoOpenInterface.h defines symbolic constants for bit
   /// masks that can be used to access the above-mentioned two bits.
   LOBOT_SENSORS_BUTTONS,

   /// Bytes 13 and 14 are a two byte value (high byte first) specifying
   /// the distance (in mm) traveled by the robot since the last time its
   /// encoders were queried. Positive values indicate forward motion,
   /// negative values indicate backward motion. This sensor's value
   /// will lie in the range [-500, 500].
   //@{
   LOBOT_SENSORS_DISTANCE_HI,
   LOBOT_SENSORS_DISTANCE_LO,
   //@}

   /// Bytes 15 and 16 are a two byte value (high byte first) specifying
   /// the angle (in degrees) turned by the robot since the last time
   /// its encoders were queried. Positive value indicate
   /// counterclockwise turns, negative values indicate clockwise turns.
   //@{
   LOBOT_SENSORS_ANGLE_HI,
   LOBOT_SENSORS_ANGLE_LO,
   //@}

   /// Byte #17 specifies the robot's charging state.
   ///
   /// DEVNOTE: LoOpenInterface.h provides symbolic constants for the
   /// different values this sensor can take on.
   LOBOT_SENSORS_CHARGING_STATE,

   /// Bytes 18 and 19 specify the voltage (in mV; high byte first)
   /// across the robot's batteries.
   //@{
   LOBOT_SENSORS_VOLTAGE_HI,
   LOBOT_SENSORS_VOLTAGE_LO,
   //@}

   /// Bytes 20 and 21 specify the current (in mA; high byte first)
   /// flowing into or out of the robot's batteries. Negative values
   /// indicate current being drawn from the batteries (as would happen
   /// when the robot is being used); positive values indicate current
   /// flowing into the battery (as during charging).
   //@{
   LOBOT_SENSORS_CURRENT_HI,
   LOBOT_SENSORS_CURRENT_LO,
   //@}

   /// Byte #22 is the battery temperature in degrees Celcius. This is a
   /// signed 8-bit integer that can take on values in the range -128 to
   /// +127.
   LOBOT_SENSORS_BATTERY_TEMP,

   /// Bytes 23 and 24 specify the battery charge (in mAh; high byte
   /// first) of the robot's batteries. The charge value will decrease
   /// when the robot is in use and its batteries get depleted; it will
   /// increase when the robot is being recharged.
   //@{
   LOBOT_SENSORS_BATTERY_CHARGE_HI,
   LOBOT_SENSORS_BATTERY_CHARGE_LO,
   //@}

   /// Bytes 25 and 26 specify the estimated charge capacity of the
   /// Create's battery (in mAh; high byte first) of the robot's
   /// batteries.
   //@{
   LOBOT_SENSORS_BATTERY_CAPACITY_HI,
   LOBOT_SENSORS_BATTERY_CAPACITY_LO,
   //@}

   /// The 27th and 28th bytes of sensor data contain a two-byte value
   /// indicating the strength of the wall sensor's signal. The high byte
   /// is sent first, then the low byte. The range of this sensor value
   /// is from zero to 4095.
   //@{
   LOBOT_SENSORS_WALL_SIGNAL_HI,
   LOBOT_SENSORS_WALL_SIGNAL_LO,
   //@}

   /// Bytes 29 through 36 contain byte pairs (high byte first)
   /// specifying the signal strengths of the cliff sensors. The range of
   /// each of these signals is from zero to 4095.
   //@{
   LOBOT_SENSORS_CLIFF_LEFT_SIGNAL_HI,
   LOBOT_SENSORS_CLIFF_LEFT_SIGNAL_LO,
   LOBOT_SENSORS_CLIFF_FRONT_LEFT_SIGNAL_HI,
   LOBOT_SENSORS_CLIFF_FRONT_LEFT_SIGNAL_LO,
   LOBOT_SENSORS_CLIFF_FRONT_RIGHT_SIGNAL_HI,
   LOBOT_SENSORS_CLIFF_FRONT_RIGHT_SIGNAL_LO,
   LOBOT_SENSORS_CLIFF_RIGHT_SIGNAL_HI,
   LOBOT_SENSORS_CLIFF_RIGHT_SIGNAL_LO,
   //@}

   /// Byte #37 specifies the state of the digital inputs on the 25-pin
   /// Cargo Bay Connector sent as individual bits (0 = low, 1 = high,
   /// i.e., 5V).
   ///
   /// DEVNOTE: LoOpenInterface.h does not yet provide symbolic bit
   /// masks for accessing these bits. Refer to the Open Interface
   /// documentation for the gory details.
   LOBOT_SENSORS_CARGO_BAY_DIGITAL_INPUTS,

   /// Bytes 38 and 39 specify the 10-bit vaue of the analog signal on
   /// the 25-pin cargo Bay Connector. The high byte is returned first.
   /// A value of zero stands for 0V; a value of 1023 is for 5V;
   /// intermediate values are for intermediate voltages.
   ///
   /// NOTE: The analog input is on pin #4 of the Cargo Bay Connector.
   //@{
   LOBOT_SENSORS_CARGO_BAY_ANALOG_SIGNAL_HI,
   LOBOT_SENSORS_CARGO_BAY_ANALOG_SIGNAL_LO,
   //@}

   /// Byte #40 specifies the robot's connection to the different
   /// charging sources. Bit #0 is for the internal charger and bit #1
   /// for the Home Base.
   ///
   /// DEVNOTE: LoOpenInterface.h provides symbolic bit masks for
   /// accessing these bits.
   LOBOT_SENSORS_CHARGING_SOURCES,

   /// Byte #41 specifies the Open Interface operating mode, which can
   /// be either Off, Passive, Safe or Full.
   ///
   /// DEVNOTE: LoOpenInterface.h provides symbolic constants for these
   /// modes.
   LOBOT_SENSORS_OI_MODE,

   /// Byte #42 specifies the currently selected song number.
   LOBOT_SENSORS_SONG_NUMBER,

   /// Byte #43 is flag indicating whether the robot is currently
   /// playing a song or not.
   LOBOT_SENSORS_SONG_PLAYING,

   /// Byte #44 specifies the number of stream packets.
   LOBOT_SENSORS_NUM_STREAM_PACKETS,

   /// Bytes 45 and 46 are a two byte value (high byte first) indicating
   /// the most recent drive speed requested by the high level. This is
   /// a signed 16-bit integer that can range from -500 to +500.
   //@{
   LOBOT_SENSORS_REQUESTED_SPEED_HI,
   LOBOT_SENSORS_REQUESTED_SPEED_LO,
   //@}

   /// Bytes 47 and 48 are a two byte value (high byte first) indicating
   /// the most recent turn radius requested by the high level. This
   /// sensor's value will be in the range [-2000, 2000] or one of the
   /// special case values for in-place turns or driving straight.
   ///
   /// DEVNOTE: LoOpenInterface.h provides symbolic constants for the
   /// special case values mentioned above.
   //@{
   LOBOT_SENSORS_REQUESTED_RADIUS_HI,
   LOBOT_SENSORS_REQUESTED_RADIUS_LO,
   //@}

   /// Bytes 49 and 50 are a two byte value (high byte first) indicating
   /// the drive speed requested by the high level for the right wheel.
   /// This sensor's value will be in the range [-500, 500].
   //@{
   LOBOT_SENSORS_REQUESTED_RIGHT_SPEED_HI,
   LOBOT_SENSORS_REQUESTED_RIGHT_SPEED_LO,
   //@}

   /// Bytes 51 and 52 are a two byte value (high byte first) indicating
   /// the drive speed requested by the high level for the left wheel.
   /// This sensor's value will be in the range [-500, 500].
   //@{
   LOBOT_SENSORS_REQUESTED_LEFT_SPEED_HI,
   LOBOT_SENSORS_REQUESTED_LEFT_SPEED_LO,
   //@}

   /// Byte 53 is an extra byte not part of the Open Interface. This byte
   /// holds a flag indicating whether or not the distance and angle
   /// packets include the actual amount spun and any unintentional
   /// translation in response to the most recent SPIN command sent by
   /// the high-level controller.
   ///
   /// DEVNOTE: The reason we need this extra byte is as follows. When
   /// the high level sends a SPIN command to have the robot turn
   /// in-place, the robot may actually end up spinning by an amount that
   /// differs slightly from the requested cw/ccw turn due to actuation
   /// errors and timing limitations in the low-level controller.
   /// Additionally, the robot may also experience a small amount of
   /// translation while rotating in-place due to wheel slippage and
   /// other such effects.
   ///
   /// Ideally, the low-level controller would then report the actual
   /// amount of rotation and any unintended translation via a SPIN
   /// acknowledgement. Unfortunately, the low-level controller's current
   /// design makes it hard to implement such an acknowledgement message
   /// because the drive module implements the response to the SPIN
   /// command but has no means of initiating an acknowledgement.
   /// Therefore, it requests the sensors module to record the actual
   /// amount of rotation and translation and tack these amounts on to
   /// the distance and angle reported by the encoders the next time the
   /// main loop requests a full sensor sweep.
   ///
   /// This extra byte in the sensors packet lets the high level know
   /// that the encoder packet it has just received includes the response
   /// to a recent SPIN command sent by it to the low level, in effect,
   /// acting as a crude sort of ACK_SPIN.
   ///
   /// DEVNOTE 2: Even if the low-level controller were able to send an
   /// ACK_SPIN in response to a CMD_SPIN, the high-level controller at
   /// present (circa August 2010) ignores all ACKs except for ACK_READY
   /// and ACK_SENSORS. Consequently, it does make some sense for the low
   /// level to tack on the effects of a CMD_SPIN to an ACK_SENSORS to
   /// ensure that the high level sees the actual amount of rotational
   /// and translational motion caused by a SPIN command.
   ///
   /// If we were to avoid this bit of hackery, it would become necessary
   /// instead to respond to all low-level ACKs and setup some sort of
   /// callback mechanism to report odometry updates to all interested
   /// high level modules (e.g., localization).
   ///
   /// Needless to say, implementing an extra flag sent as part of the
   /// low-level sensor packet is a lot less work and, therefore, we
   /// adopt this solution now in the interests of minimizing development
   /// time.
   ///
   /// DEVNOTE 3: Eventually, when we redesign the low-level controller
   /// and the interface to it in the high level, it would be a good idea
   /// to have the main module act as a general communication buffer that
   /// queues incoming commands from the high level and outgoing
   /// acknowledgements from the low level and to then allow all
   /// low-level modules to queue relevant acknowledgements. Thus, when a
   /// SPIN command comes in, the drive module could handle it and queue
   /// an ACK_SPIN in response without any trouble.
   ///
   /// Of course, the high-level thread/module interfacing with the
   /// low-level controller will also have to be redesigned to accept all
   /// low-level ACKs and implement a proper observer pattern to inform
   /// interested parties about the latest odometry updates, etc.
   LOBOT_SENSORS_SPIN_FLAG,
} ;

/// These enumerations define bit masks for the rear bumpers.
enum {
   /// Instead of adding a new sensor data byte for the rear bumpers to
   /// be sent as part of the SENSORS_ACK message, we simply use the
   /// unused bits of the bumps + wheel drops data byte. These bit masks
   /// can be used to access the states of the rear bumpers when the
   /// bumps data byte of the SENSORS_ACK message is being used.
   //@{
   LOBOT_BUMP_REAR_LEFT   = 0x40,
   LOBOT_BUMP_REAR_RIGHT  = 0x20,
   LOBOT_BUMP_REAR_BOTH   = 0x60,
   LOBOT_BUMP_REAR_EITHER = 0x60,
   //@}

   /// When the low-level controller sends the BUMPS_ACK message to the
   /// high level, it packages the states of the front and rear bump
   /// sensors into lower four bits of the first data byte.
   /// LoOpenInterface.h defines bit masks for the (built-in) front
   /// bumpers. These bit masks specify the locations of the rear left
   /// and right bump sensors for the BUMPS_ACK message.
   //@{
   LOBOT_BUMP_ACK_REAR_LEFT   = 0x08,
   LOBOT_BUMP_ACK_REAR_RIGHT  = 0x04,
   LOBOT_BUMP_ACK_REAR_BOTH   = 0x0C,
   LOBOT_BUMP_ACK_REAR_EITHER = 0x0C,
   //@}
} ;

/*---------------------------------------------------------------------*/

#ifdef __cplusplus
} // end of extern "C"
#endif

#endif
