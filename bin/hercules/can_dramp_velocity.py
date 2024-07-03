import can
import time
import logging
import argparse

def digital_ramp_test(bus,reverse=False):
        print("Starting to send a message every 1.5ms. Initial data is four consecutive 1.5ms")
        msg = can.Message(arbitration_id=0x100, data=[0,0,0,0], is_extended_id=False)
        task = bus.send_periodic(msg, 0.0015)
        if not isinstance(task, can.ModifiableCyclicTaskABC):
            print("This interface doesn't seem to support modification")
            task.stop()
            return

        if reverse == False :
            #Accelation phase
            print("\nAccelation phase\n")
            can_msg = 0x0000
            time.sleep(0.0015)
            while msg.data[1] != 0x25:
                can_msg += 0x0001 * 5
                msg.data[0] = can_msg & 0xFF
                msg.data[1] = (can_msg >> 8) & 0xFF
                task.modify_data(msg)
                time.sleep(0.0015)

            #Constant phase
            print("\nConstant phase\n")
            time.sleep(0.0015)
            for i in range(0, 1500):
                task.modify_data(msg)
                time.sleep(0.0015)

            #Breaking phase
            print("\nBreaking phase\n")
            time.sleep(0.0015)
            while can_msg != 0x0000:
                can_msg -= 0x0001 * 5
                msg.data[0] = can_msg & 0xFF
                msg.data[1] = (can_msg >> 8) & 0xFF
                task.modify_data(msg)
                time.sleep(0.0015)
            task.stop()
            print("done")
        
        else :  
            #Accelation phase
            print("\nAccelation phase\n")
            can_msg = 0xFFFF
            time.sleep(0.0015)
            while msg.data[1] != 0xDA:
                can_msg -= 0x0001 * 5
                msg.data[0] = can_msg & 0xFF
                msg.data[1] = (can_msg >> 8) & 0xFF
                task.modify_data(msg)
                time.sleep(0.0015)

            #Constant phase
            print("\nConstant phase\n")
            time.sleep(0.0015)
            for i in range(0, 1500):
                task.modify_data(msg)
                time.sleep(0.0015)

            #Breaking phase
            print("\nBreaking phase\n")
            time.sleep(0.0015)
            while can_msg !=  0xFFFF:
                can_msg += 0x0001 * 5
                msg.data[0] = can_msg & 0xFF
                msg.data[1] = (can_msg >> 8) & 0xFF
                task.modify_data(msg)
                time.sleep(0.0015)
            task.stop()
            print("done")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('Channel', metavar='channel', type=str, help='the channel to be used')
    parser.add_argument('--reverse', action='store_true', help='reverse the direction of the ramp')
    args = parser.parse_args()
    with can.interface.Bus(bustype='socketcan', channel=args.Channel, bitrate=500000) as bus:
        digital_ramp_test(bus, reverse=args.reverse)
    time.sleep(2)

if __name__ == "__main__":
    main()
