from mlagents_envs.environment import UnityEnvironment
import numpy as np
import mlagents_envs, mlagents
import cv2

print(mlagents_envs.__version__)
env = UnityEnvironment(file_name="C:\\Users\\dvijk\\uneven-terrain-driver\\Build-windows-new\\uneven-terrain-driver", seed=1, side_channels=[])
# Start interacting with the environment.

env.reset()

behavior_name = list(env.behavior_specs)[0]

# Reset the environment
env_info = env.reset()
action_size = env.behavior_specs[behavior_name].action_spec.continuous_size
random_action = np.random.uniform(-1, 1, size=(1, action_size))
print({behavior_name: random_action})
# Step the environment
for i in range(12) :
    env.step()
env_info = env.get_steps(behavior_name)
observations = np.floor(env_info[0].obs[0][0,:,:,0]*255)
# observations = env_info.vector_observations
observations = observations.astype(np.uint8)
print(np.array(observations).shape,np.max(observations),np.min(observations))

cv2.imwrite('output_image.png', observations)
print("Saved")