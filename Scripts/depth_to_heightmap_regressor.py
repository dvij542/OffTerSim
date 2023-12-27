import numpy as np
import torch
from mlagents_envs.environment import UnityEnvironment,ActionTuple
from mlagents_envs.base_env import ActionSpec
# import cv2
import mlagents_envs, mlagents
import time

env = UnityEnvironment(file_name="Build-ubuntu-easy/exec", seed=1, side_channels=[])
env.reset()

behavior_name = list(env.behavior_specs)[0]
action_size = env.behavior_specs[behavior_name].action_spec.continuous_size
random_action = np.random.uniform(-1, 1, size=(1, action_size))

# print({behavior_name: random_action})
# Step the environment
# Define discrete action specs for each action
action_spec_1 = ActionSpec.create_discrete(3)
action_spec_2 = ActionSpec.create_discrete(2)

# Combine the action specs into a dictionary
# action_specs = {
#     "action_1": action_spec_1,
#     "action_2": action_spec_2
# }

# Set the actions for the agent
for i in range(1200) :
    # env.set_actions(behavior_name,np.array([[0,1]]))
    # actions = {
    #     "action_1": [2],  # Assuming action index 2 is chosen for the first action
    #     "action_2": [1]   # Assuming action index 1 is chosen for the second action
    # }
    actions = ActionTuple(None,np.array([[1,1]]))
    env.set_actions(behavior_name, actions)
    env.step()
    time.sleep(0.01)

env_info = env.get_steps(behavior_name)
depth_image = env_info[0].obs[0][0,:,:,0]

vector_observations = env_info[0].obs[1]
vals_to_predict = vector_observations[-55:]
# observations = observations.astype(np.uint8)
# print(np.array(observations).shape,np.max(observations),np.min(observations))

# cv2.imwrite('output_image.png', observations)
print("Saved")