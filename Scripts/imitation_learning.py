import os
# Set the GPU(s) to be used
os.environ["CUDA_VISIBLE_DEVICES"] = "0"
import numpy as np
import torch
from mlagents_envs.environment import UnityEnvironment,ActionTuple
from mlagents_envs.base_env import ActionSpec, TerminalSteps
# import cv2
import mlagents_envs, mlagents
import time
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim


# Hyperparameters
input_dim = 4096+12
output_dim = 3
n_epsiodes = 1000
buffer_size = 128
epsilon = 0.1
factor = 1

class PolicyNN(nn.Module):
    def __init__(self, input_dim, hidden_dims, output_dim, is_classifier = True):
        super(PolicyNN, self).__init__()

        # Define input layer
        self.layers = [nn.Linear(input_dim, hidden_dims[0])]
        self.layers.append(nn.ReLU())
        
        # Define hidden layers
        for i in range(len(hidden_dims) - 1):
            self.layers.extend([nn.Linear(hidden_dims[i], hidden_dims[i + 1]), nn.ReLU()])

        # Define output layer
        self.layers.append(nn.Linear(hidden_dims[-1], output_dim))

        # Create the sequential model
        self.model = nn.Sequential(*self.layers)
        self.is_classifier = is_classifier
            


    def forward(self, x):
        # Forward pass through the network
        x = self.model(x)
        
        # Apply softmax activation to the output
        # if self.is_classifier :
        #     x = F.softmax(x, dim=1)
        

        return x

class DepthToHeightNN(nn.Module):
    def __init__(self, input_dim, hidden_dims, output_dim, is_classifier = True):
        super(DepthToHeightNN, self).__init__()

        # Define input layer
        self.layers = [nn.Linear(input_dim, hidden_dims[0])]
        self.layers.append(nn.ReLU())
        
        # Define hidden layers
        for i in range(len(hidden_dims) - 1):
            self.layers.extend([nn.Linear(hidden_dims[i], hidden_dims[i + 1]), nn.ReLU()])

        # Define output layer
        self.layers.append(nn.Linear(hidden_dims[-1], output_dim))

        # Create the sequential model
        self.model = nn.Sequential(*self.layers)
        self.is_classifier = is_classifier
        self.sigmoid = nn.Sigmoid()
            


    def forward(self, x):
        # Forward pass through the network
        x = self.model(x)
        
        # Apply softmax activation to the output
        # if self.is_classifier :
        #     x = F.softmax(x, dim=1)

        return 6.*self.sigmoid(x) - 3.


def expert_policy(heights) :
    ranges = [[0,4],[4,7],[7,11]]
    cum_heights = []
    for i in range(3) :
        curr_cum_height = 0
        for j in range(ranges[i][0],ranges[i][1]) :
            curr_cum_height += heights[i]/(ranges[i][1]-ranges[i][0])
        cum_heights.append(-curr_cum_height)
    # print(cum_heights)
    return 2 - np.argmax(cum_heights)

def expert_policy_1(heights) :
    dir_ind = np.argmax(-heights)
    if dir_ind < 4 :
        return 2
    elif dir_ind < 7 :
        return 1
    else :
        return 0 

# Define policy network
policy = PolicyNN(input_dim=input_dim,hidden_dims=[2048,1024,512,128],output_dim=output_dim).cuda()
height_regressor = DepthToHeightNN(input_dim=input_dim,hidden_dims=[2048,1024,512,128],output_dim=55).cuda()
# print(policy)
env = UnityEnvironment(file_name="./Build-ubuntu/exec", seed=2, side_channels=[], no_graphics=True)
env.reset()

behavior_name = list(env.behavior_specs)[0]
action_size = env.behavior_specs[behavior_name].action_spec.continuous_size
random_action = np.random.uniform(-1, 1, size=(1, action_size))

# Define Binary Cross Entropy Loss

criterion = nn.CrossEntropyLoss(reduce=False)
criterion_hr = nn.MSELoss()

# Define an optimizer, e.g., Stochastic Gradient Descent (SGD)
optimizer = optim.SGD(policy.parameters(), lr=0.001)
optimizer_hr = optim.SGD(height_regressor.parameters(), lr=0.001)

# Example training loop
num_epochs = 10


curr_episode = 0
i = 0
i_ = 0
buffer = torch.zeros((buffer_size,input_dim)).cuda()
buffer_outs = torch.zeros((buffer_size,)).cuda().to(torch.long)
buffer_wts = torch.zeros((buffer_size,1)).cuda()
buffer_hts = torch.zeros((buffer_size,55)).cuda()
buffer_ = torch.zeros((buffer_size,input_dim)).cuda()
curr_per_iter = 0
while(True) :
    # print(i)
    # Get observations
    start_time = time.time()
    env_info = env.get_steps(behavior_name)
    depth_image = env_info[0].obs[0][0,:,:,0]
    if len(env_info[1]) > 0 :
        # print("Episode finished?")
        curr_per_iter = 0
        curr_episode += 1
        torch.save(policy,'checkpoint'+str(curr_episode)+'.pt')
    
    # print("Initialization time:", (time.time()-start_time))
    start_time = time.time()
    # print(len(env_info[1]))
    # print(depth_image.shape)
    vector_observations = env_info[0].obs[1]
    # print(vector_observations.shape)
    # print(vector_observations)
    heights = vector_observations[0,-55:].reshape((5,11))
    # print(heights)
    expert_cmd = expert_policy(heights[1,:])

    # print("Expert policy time:", (time.time()-start_time))
    start_time = time.time()
    n_collisions = vector_observations[0,11]
    print("n_collisions:",n_collisions)
    # one_hot_encoded = np.eye(3)[expert_cmd]
    # one_hot_tensor = torch.from_numpy(one_hot_encoded).float().cuda()
    print("Dpeth sum = ", np.sum(depth_image))
    inputs = torch.from_numpy(np.concatenate((depth_image.flatten(),vector_observations[0,:-55]))).cuda()
    if n_collisions < 0.1 : # Update only when no collisions
        i += 1
        buffer[1:,:] = buffer[:-1,:].clone()
        buffer[0,:] = inputs
        buffer_outs[1:] = buffer_outs[:-1].clone()
        buffer_outs[0] = expert_cmd
        buffer_wts[1:,:] = buffer_wts[:-1,:].clone()
        buffer_wts[0,0] = max(0,factor*np.max(heights[:3,:])+epsilon) #- np.min(cum_heights)
    
    if n_collisions < 0.1 and max(0,factor*np.max(heights[:3,:])+epsilon) > 1.5: # Update only when no collisions    
        i_ += 1
        buffer_[1:,:] = buffer_[:-1,:].clone()
        buffer_[0,:] = inputs
        buffer_hts[1:,:] = buffer_hts[:-1,:].clone()
        buffer_hts[0,:] = torch.from_numpy(vector_observations[0,-55:]).cuda()
        print('Updating buffer')
        
    # print(buffer_outs)
    inputs_torch = buffer[:min(i,buffer_size),:]
    outputs_gt_torch = buffer_outs[:min(i,buffer_size)]
    outputs = policy(inputs_torch)
    loss = torch.mean(buffer_wts*criterion(outputs, outputs_gt_torch))
    # print(torch.argmax(outputs,dim=1))
    print("Loss :", curr_episode, i, float(loss.item()),torch.sum(torch.argmax(outputs,dim=1)==outputs_gt_torch))
    
    # Backward pass and optimization
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()
    # print("Policy update time:", (time.time()-start_time))
    
    if i_ > 10 :
        inputs_torch_ = buffer_[:min(i_,buffer_size),:]
        heights_pred = height_regressor(inputs_torch_)
        heights_gt_torch = buffer_hts[:min(i_,buffer_size)]
        loss_hr = criterion_hr(heights_pred, heights_gt_torch)
        print("HR Loss :", curr_episode, i, float(loss_hr.item()))
        optimizer_hr.zero_grad()
        loss_hr.backward()
        optimizer_hr.step()
    
        if max(0,factor*np.max(heights[:3,:])+epsilon) > 1.5 and i > 5000 and curr_per_iter < 100:
            curr_per_iter += 1
            hts_comp = np.stack((buffer_hts[0,:].cpu().numpy(),heights_pred[0,:].detach().cpu().numpy()))
            np.savetxt('eg_comps/eg_'+str(curr_per_iter)+'_'+str(curr_episode)+'.txt',hts_comp)
    
    start_time = time.time()
    # print(int(torch.argmax(outputs[0,:]).cpu().detach().numpy()))
    # Set actions
    if curr_episode < 100 :
        actions = ActionTuple(None,np.array([[expert_cmd,1]]))
    else :
        actions = ActionTuple(None,np.array([[int(torch.argmax(outputs[0,:]).cpu().detach().numpy()),1]]))
    env.set_actions(behavior_name, actions)
    env.step()
    # print("Environment update time:", (time.time()-start_time))
    
    # time.sleep(0.01)

# observations = observations.astype(np.uint8)
# print(np.array(observations).shape,np.max(observations),np.min(observations))

# cv2.imwrite('output_image.png', observations)
print("Saved")