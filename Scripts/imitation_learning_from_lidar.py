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
import random
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
import math
from PIL import Image
from scipy.interpolate import griddata
import onnx
import onnxruntime

########## INSTRUCTIONS ##########
# 1. Add reward calculation, refer to Assets/KartAgent.cs
# Observation inputs:-
#   0-15. Depth observations from LIDAR (angles -15 to +15 degrees, resolution 2 degrees): 182 
#           Format:- (0,2,-2,4,-4,...,90,-90) degrees and each input contains (mask,range/5.)
#   16. State: 12 (speed,vx,vy,vz(0.),roll(in degrees)/90.,omega(yaw rate),pitch(in degrees)/90.,
#                  last throttle,last brake,heading angle error,lateral displacement,n_collisions(0))

# Hyperparameters
input_dim = 4096+12
output_dim = 3
n_epsiodes = 1000
buffer_size = 128
epsilon = 0.1
factor = 1
n_divs = 10 # No of divisions for data balancing
using_gt_hts = False

# Load the ONNX model
POLICY_LOC = "./results/without_lidar_attempt5/ArcadeDriver/ArcadeDriver-1209008.onnx"
model = onnx.load(POLICY_LOC)
sess = onnxruntime.InferenceSession(POLICY_LOC)

# Input data (replace this with your own input data)
input_data = np.random.rand(1, 67).astype(np.float32)
input_mask = np.array([[1.,1.,1.,1.,1.]]).astype(np.float32)

# Run inference
input_name = sess.get_inputs()[0].name
mask_name = sess.get_inputs()[1].name
print(mask_name)
# exit(0)
output_name = sess.get_outputs()[2].name
for output in sess.get_outputs():
    print(output.name)
    print(output.shape)
# Run the model with the input data
# result = sess.run([output_name], {input_name: input_data, input_mask: input_mask})

# Run the model with the input data
result = sess.run([output_name], {input_name: input_data, mask_name: input_mask})

# Print the inference result
print("Inference result:", result[0])

# # Get the input details
# input_details = model.graph.input

# # Print information about input size and shape
# for input_detail in input_details:
#     input_name = input_detail.name
#     input_type = input_detail.type.tensor_type
#     input_shape = input_type.shape.dim
    
#     print(f"Input name: {input_name}")
#     print(f"Input shape: {[dim.dim_value for dim in input_shape]}")
#     print("---")

# print(torch.load(POLICY_LOC))
# exit(0)

# TODO: Read this
# Output format:-
# 0: steering (3 classes): left, straight, right
# 1: acceleration (2 classes): -1, 1
# hr means height regressor

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

def interpolate_point(a, b, data):
    # Extract x, y, and f(x, y) values from the data
    x = data[:, 0]
    y = data[:, 1]
    values = data[:, 2]

    # Perform 2D linear interpolation
    interpolated_value = griddata((x, y), values, (a, b), method='nearest')
    # print(interpolated_value)
    return interpolated_value

def expert_policy(heights, network_availale = True) :
    if network_availale :
        print(heights.shape,input_data.shape,input_mask.shape)
        result = sess.run([output_name], {input_name: heights, mask_name: input_mask})
        print("Inference result:", result[0])
        return result[0][0][0]
    ranges = [[0,4],[4,7],[7,11]]
    cum_heights = []
    for i in range(3) :
        curr_cum_height = 0
        for j in range(ranges[i][0],ranges[i][1]) :
            curr_cum_height += heights[j]/(ranges[i][1]-ranges[i][0])
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

# Define policy network (TODO: Define the network architecture)
policy = PolicyNN(input_dim=input_dim,hidden_dims=[2048,1024,512,128],output_dim=output_dim).cuda()

# Define height regressor network (TODO: Define the network architecture for sarthak to predict the privileged heights)
height_regressor = DepthToHeightNN(input_dim=input_dim,hidden_dims=[2048,1024,512,128],output_dim=55).cuda()

# Load the regressor network if needed to continue training
height_regressor = torch.load('checkpoints/checkpoint_hr99.pt')
# height_regressor.eval()

# Using the with_liar observation environment
env = UnityEnvironment(file_name="with_lidar_new/with_lidar", seed=2, side_channels=[],no_graphics=True)
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

# Example training loop, buffer for storing the data
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

dist_freqs_inv = np.ones(n_divs)
gap_factor = 3./n_divs
n_corrects = 1
n_total = 1

def get_sample_points(pcl,angle_max=37.5*math.pi/180.,ld=2.,npts=11) :
    # Outputs the heights sampled at 55 points ahead of the car given the point cloud
    pcl = np.array(pcl)
    print(pcl.shape)
    if np.sum(pcl) < 0.001 :
        return np.array([0.]*11)
    pts_int = []
    for a in np.arange(-angle_max,angle_max-0.001,2*angle_max/npts) :
        pt = (ld*math.sin(a),ld*math.cos(a))
        # print(arr.shape)
        try:
            z = interpolate_point(pt[0],pt[1],np.array([pcl[:,0],pcl[:,1],pcl[:,2]]).T)
        except:
            print("Not feasible solution")
            return np.array([0.]*11)
        # print(a,z)
        x = pt[0]
        y = pt[1]
        pts_int.append(z)
    pts_int = np.array(pts_int)[::-1]
    # print(pts_int)
    return pts_int

def convert_to_pt_cloud(img,ang_wide=75.,dist_factor=6.) :
    # Convert the depth image to point cloud
    pts = []
    for i in range(img.shape[0]) :
        for j in range(img.shape[1]) :
            angle_vert = -(i - img.shape[0]/2.)*(ang_wide/img.shape[0])
            angle_hor = (j - img.shape[1]/2.)*(ang_wide/img.shape[0])
            y = img[i,j]*dist_factor
            x = y*math.tan(angle_hor*math.pi/180.)
            z = y*math.tan(angle_vert*math.pi/180.)
            pts.append([x,y,z])
    return pts

curr_per_iter_ = 0
while(True) :
    print("benchod")
    # Get observations
    start_time = time.time()
    env_info = env.get_steps(behavior_name)
    if len(env_info[0].obs) == 0 :
        print("No observations")
    
    if len(env_info[1]) > 0 :
        # print("Episode finished?")
        curr_per_iter = 0
        curr_per_iter_ = 0
        curr_episode += 1
        torch.save(policy,'checkpoint'+str(curr_episode)+'.pt')
        torch.save(height_regressor,'checkpoint_hr'+str(curr_episode)+'.pt')
    
    # print("Initialization time:", (time.time()-start_time))
    start_time = time.time()
    vector_observations = env_info[0].obs[-1]
    # print(vector_observations)
    # exit(0)
    heights = vector_observations[0,-55:].reshape((5,11))
    
    # print(np.min(depth_image),np.max(depth_image))
    # im.save('buf_image.png')
    # np.savetxt('haha.txt',normalized_array, fmt='%i')
    # TODO: env_info[0].obs[-1] has length 67, first 12: inertial state and last 55: privileged depth
    # TODO: Get the lidar info from env_info[0].obs[:-1]
    # TODO for sarthak: Use height_regressor to get privileged heights

    # 6000 best reward. 2k-3k also good 
    # inputs = torch.from_numpy(np.concatenate((depth_image.flatten(),vector_observations[0,:-55]))).cuda()
    # heights_infer = height_regressor(inputs.unsqueeze(0)).detach().cpu().numpy()[0,:].reshape((5,11))
    # print(heights)
    # pcl = convert_to_pt_cloud(depth_image)
    # pts = get_sample_points(pcl)
    if using_gt_hts or curr_episode < 100:
        expert_cmd = expert_policy(vector_observations, network_availale=True)
    else :
        # TODO for sarthak: Handle this case
        # expert_cmd = expert_policy(heights_infer[1,:])
        # expert_cmd_ = expert_policy(heights[0,:])
        # expert_cmd__ = expert_policy(pts)
        # print(expert_cmd,expert_cmd__)
        # if expert_cmd == expert_cmd_ :
        #     n_corrects += 1
        n_total += 1
    # print("Expert policy time:", (time.time()-start_time))
    start_time = time.time()
    n_collisions = vector_observations[0,11]
    
    if n_collisions < 0.1 : # Update only when no collisions
        i += 1
        buffer[1:,:] = buffer[:-1,:].clone()
        # buffer[0,:] = inputs # TODO: Uncomment this when done with the input format
        buffer_outs[1:] = buffer_outs[:-1].clone()
        buffer_outs[0] = expert_cmd
        buffer_wts[1:,:] = buffer_wts[:-1,:].clone()
        buffer_wts[0,0] = max(0,factor*np.max(heights[:3,:])+epsilon) #TODO: I am weighing according to the max height difference here to balance the data and give more weight to data when near obstacles
    
    curr_val = int(min(2.9,max(0.,factor*np.max(heights[:3,:])+epsilon))/gap_factor)
    dist_freqs_inv[curr_val] = 1./(1./dist_freqs_inv[curr_val] + 1)
    # print(n_corrects/n_total)
    # print((dist_freqs_inv[curr_val]/np.sum(dist_freqs_inv)),random.random())
    if False: #curr_val > 1.5 and abs(expert_cmd-expert_cmd__) > 1.5:
        arr_gt = heights[1,:]
        arr_pred = heights_infer[1,:]
        
        

        curr_per_iter_ += 1
        np.savetxt('raw_pts/eg_'+str(curr_per_iter_)+'_'+str(curr_episode)+'.txt',pcl)
        print("Saavedddddddd",curr_per_iter_,curr_episode)

        print('Saved',curr_per_iter_,curr_episode)
        # time.sleep(3)
        # pts_gt = []
        # pts_pred = []
        # for i in range(5) :
        #     for j in range(11) :
        #         t = (-30 + 6*j)*math.pi/180.
        #         x = (i+1)*math.sin(t)
        #         y = (i+1)*math.cos(t)
        #         z1 = arr_gt[i,j]
        #         z2 = arr_pred[i,j]
        #         pts_gt.append([x,y,z1])
        #         pts_pred.append([x,y,z2])

        # pts_gt = np.array(pts_gt)
        # pts_pred = np.array(pts_pred)

        # # Create a 3D plot
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')

        # Scatter plot
        # ax.scatter(pts_gt[:,0], pts_gt[:,1], pts_gt[:,2], c='b', marker='o')
        # ax.scatter(pts_pred[:,0], pts_pred[:,1], pts_pred[:,2], c='r', marker='o')

        # Set labels
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.set_zlabel('Z')
        # plt.plot(arr_gt)
        # plt.plot(arr_pred)
        # Show the plot
        # plt.show()
        print(arr_gt)
        print(arr_pred)
        
        # time.sleep(5)
    
    if n_collisions < 0.1 and random.random() < (dist_freqs_inv[curr_val]/np.sum(dist_freqs_inv)): # Update only when no collisions    
        i_ += 1
        buffer_[1:,:] = buffer_[:-1,:].clone()
        # buffer_[0,:] = inputs # TODO: Uncomment these when done with the input format
        buffer_hts[1:,:] = buffer_hts[:-1,:].clone()
        buffer_hts[0,:] = torch.from_numpy(vector_observations[0,-55:]).cuda()
        print('Updating buffer')
    

    # TODO: Uncomment these accordingly
    # print(buffer_outs)
    # inputs_torch = buffer[:min(i,buffer_size),:]
    # outputs_gt_torch = buffer_outs[:min(i,buffer_size)]
    # outputs = policy(inputs_torch)
    # loss = torch.mean(buffer_wts*criterion(outputs, outputs_gt_torch))
    # print(torch.argmax(outputs,dim=1))
    # print("Loss :", curr_episode, i, float(loss.item()),torch.sum(torch.argmax(outputs,dim=1)==outputs_gt_torch))
    
    # Backward pass and optimization
    # optimizer.zero_grad()
    # loss.backward()
    # optimizer.step()
    # print("Policy update time:", (time.time()-start_time))
    
    if i_ > 1 :
        inputs_torch_ = buffer_[:min(i_,buffer_size),:]
        heights_pred = height_regressor(inputs_torch_)
        heights_gt_torch = buffer_hts[:min(i_,buffer_size)]
        loss_hr = criterion_hr(heights_pred, heights_gt_torch)
        # print("HR Loss :", curr_episode, i, float(loss_hr.item()))
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
        # TODO: Replace this when done with the training and want to take command from the policy instead of expert 
        actions = ActionTuple(None,np.array([[expert_cmd,1]]))
    else :
        actions = ActionTuple(None,np.array([[int(torch.argmax(outputs[0,:]).cpu().detach().numpy()),1]]))
    env.set_actions(behavior_name, actions)
    env.step()
    # print("Environment update time:", (time.time()-start_time))
    
    # time.sleep(0.01)

# observations = observations.astype(np.uint8)
# print(np.array(observations).shape,np.max(observations),np.min(observations))

print("Saved")
