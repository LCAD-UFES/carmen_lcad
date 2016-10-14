

#ifndef DQNPARAMS_H_
#define DQNPARAMS_H_

#define DQN_INPUT_LAYER_NAME "frames_input_layer"
#define DQN_FILTER_LAYER_NAME "filter_input_layer"
#define DQN_TARGET_LAYER_NAME "target_input_layer"
#define DQN_ADD_DATA_LAYER_NAME "additional_data_input_layer"

#define DQN_INPUT_BLOB_NAME "frames"
#define DQN_TARGET_BLOB_NAME "target"
#define DQN_FILTER_BLOB_NAME "filter"
#define DQN_ADD_DATA_BLOB_NAME "additional_data"
#define DQN_OUTPUT_BLOB_NAME "q_values"

#define DQN_MODE_Q_LEARNING 1
#define DQN_MODE_TOTAL_REWARD 2
#define DQN_MODE_REWARD_WITH_DECAY 2
#define DQN_MODE_INFINITY_HORIZON_DISCOUNTED_MODEL 3

#define DQN_JUST_TEST false								// "Evaluation mode: only playing a game, no updates"

#define DQN_NUM_WARMUP_TRANSITIONS 500					// "Enough amount of transitions to start learning"
#define DQN_SKIP_FRAME 0								// "Number of frames skipped"
#define DQN_TOTAL_ACTIONS 5

#define DQN_GAMMA 0.99 									// "Discount factor of future rewards (0,1]"
#define DQN_NUM_EPISODES_TO_STORE 20					// "Capacity of replay memory"

#define DQN_FRAME_DIM 100
#define DQN_FRAME_CHANNELS 3
#define DQN_NUM_INPUT_FRAMES 3

#define DQN_NUM_PAST_COMMANDS_TO_STORE 5
#define DQN_NUM_PAST_ODOMS_TO_STORE 5
#define DQN_NUM_PAST_GOAL_POSES_TO_STORE 5
#define DQN_NUM_ADDITIONAL_DATA (DQN_NUM_PAST_COMMANDS_TO_STORE + 2 * DQN_NUM_PAST_ODOMS_TO_STORE + 3 * DQN_NUM_PAST_GOAL_POSES_TO_STORE)
#define DQN_NUM_COPIES_OF_ADDITIONAL_DATA 10

#define DQN_MINI_BATCH_SIZE 1
#define DQN_NUM_COMMANDS DQN_TOTAL_ACTIONS
#define DQN_TRAINING_MODE DQN_MODE_Q_LEARNING

#define DQN_TOTAL_ADDITIONAL_DATA_SIZE (DQN_NUM_ADDITIONAL_DATA * DQN_NUM_COPIES_OF_ADDITIONAL_DATA)
#define DQN_TOTAL_FRAME_SIZE (DQN_FRAME_DIM * DQN_FRAME_DIM * DQN_FRAME_CHANNELS)
#define DQN_TOTAL_INPUT_SIZE (DQN_NUM_INPUT_FRAMES * DQN_TOTAL_FRAME_SIZE)
#define DQN_TOTAL_MINI_BATCH_SIZE (DQN_MINI_BATCH_SIZE * DQN_TOTAL_INPUT_SIZE)

#endif
