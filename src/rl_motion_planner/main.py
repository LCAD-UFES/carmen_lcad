
import carmen_comm.carmen_comm as carmen

carmen.env_init()
state = carmen.env_reset(7757732.33, -363558.89, 0.651)

while not carmen.env_done():
    print(state)
    state = carmen.env_step(1.0, 0.0)

