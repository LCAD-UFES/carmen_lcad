import can
import time
import logging
import argparse

def digital_ramp_test(bus):
        print("Starting to send a message every 1.5ms. Initial data is four consecutive 1.5ms")
        msg = can.Message(arbitration_id=0x100, data=[0,0,0,0], is_extended_id=False)
        task = bus.send_periodic(msg, 0.0015)
        if not isinstance(task, can.ModifiableCyclicTaskABC):
            print("This interface doesn't seem to support modification")
            task.stop()
            return

        #Accelation phase
        print("\nAccelation phase\n")
        can_msg = 0x1000
        for i in range(50):
            msg.data[2] = can_msg & 0xFF
            msg.data[3] = (can_msg >> 8) & 0xFF
            task.modify_data(msg)
            time.sleep(0.15)

        #Deceleration phase
        print("\nDeceleration phase\n")
        time.sleep(0.0015)
        can_msg = 0xEFFF
        for i in range(50):
            msg.data[2] = can_msg & 0xFF
            msg.data[3] = (can_msg >> 8) & 0xFF
            task.modify_data(msg)
            time.sleep(0.15)
        task.stop()
        print("done")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('Channel', metavar='channel', type=str, help='the channel to be used')
    args = parser.parse_args()
    with can.interface.Bus(bustype='socketcan', channel=args.Channel, bitrate=500000) as bus:
        digital_ramp_test(bus)
    time.sleep(2)

if __name__ == "__main__":
    main()
