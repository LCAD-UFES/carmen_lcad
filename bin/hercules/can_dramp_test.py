import can
import time
import logging

def single_msg_send(bus):
    msg = can.Message(arbitration_id=0x425, data=[0,0,0,0], is_extended_id=False)
    bus.send(msg)

def simple_periodic_send(bus):
    """
    Sends a message every 20ms with no explicit timeout
    Sleeps for 2 seconds then stops the task.
    """
    print("Starting to send a message every 200ms for 2s")
    msg = can.Message(
        arbitration_id=0x123, data=[1, 2, 3, 4, 5, 6], is_extended_id=False
    )
    task = bus.send_periodic(msg, 0.20)
    assert isinstance(task, can.CyclicSendTaskABC)
    time.sleep(2)
    task.stop()
    print("stopped cyclic send")

def test_periodic_send_with_modifying_data(bus):

    print("Starting to send a message every 1.5ms. Initial data is four consecutive 1.5ms")
    msg = can.Message(arbitration_id=0x100, data=[0,0,0,0], is_extended_id=False)
    task = bus.send_periodic(msg, 0.0015)
    if not isinstance(task, can.ModifiableCyclicTaskABC):
        print("This interface doesn't seem to support modification")
        task.stop()
        return
    
    #Accelation phase
    print("\nAccelation phase\n")
    can_msg = 0x0000
    #can_msg = 0xFFFF
    time.sleep(0.0015)
    while msg.data[1] != 0x30:
    #while msg.data[1] != 0xCF:
        can_msg += 0x0001 * 5
        #can_msg -= 0x0001 * 5
        msg.data[0] = can_msg & 0xFF
        msg.data[1] = (can_msg >> 8) & 0xFF
        task.modify_data(msg)
        time.sleep(0.0015)
        
    #Breaking phase
    print("\nBreaking phase\n")
    time.sleep(0.0015)
    while can_msg !=  0x0000:
    #while can_msg !=  0xFFFF:
        can_msg -= 0x0001 * 5
        #can_msg += 0x0001 * 5
        msg.data[0] = can_msg & 0xFF
        msg.data[1] = (can_msg >> 8) & 0xFF
        task.modify_data(msg)
        time.sleep(0.0015)
    task.stop()
    print("done")

def main():

    #reset_msg = can.Message(arbitration_id=0x00, data=[0, 0, 0, 0, 0, 0], is_extended_id=False)

    with can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000) as bus:
        test_periodic_send_with_modifying_data(bus)
    time.sleep(2)
    #with can.interface.Bus(bustype='socketcan', channel='vcan0', bitrate=500000) as bus:
    #    test_periodic_send_with_modifying_data(bus)
    #time.sleep(2)

if __name__ == "__main__":
    main()
