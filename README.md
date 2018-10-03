# Object_estimator
This repository is intended for a package required to estimate the internal state of a grasped object by a manipulator. This includes a binary classifier using log-likelihood ratio from hidden Markov models (HMMs), a data preprocessor, and a topic recording script. 

This repository is tested with a Baxter robot in RRG, MIT. For deployment, users have to record a set of training data rosbags, train an HMM, and run a ros-service with data topics.

## Installation
- Install HMM library following http://ghmm.sourceforge.net/installation.html 
  (See my old installation script: https://github.com/pidipidi/dpark_setup/blob/master/script/deprecated/ml_ghmm_install.sh)
- Install dependencies
~~~~bash
pip install -r requirements.txt
~~~~
- Compile this package
~~~~bash
catkin_make
~~~~
- Download test data
~~~~bash
roscd object_estimator
cd data
wget https://www.dropbox.com/s/vj8g3ftplq0lru6/object_estimator.tar.gz
tar -xvzf object_estimator.tar.gz
~~~~

## Test
- Preprocess the collected data. 
~~~~bash
roslaunch object_estimator full_cup_estimation.launch en_preprocess:=true plot:=true
~~~~

- Then, run the estimation service node. You can see a service node "/object_state/left". You can also specify a service node name in the launch file.
~~~~bash
roslaunch object_estimator full_cup_estimation.launch 
~~~~

- Option) We can call the service while playing a rosbag and check the classification result.
~~~~bash
rosbag play data/s50_heavy/test_2018-04-26-22-22-53.bag
rosservice call /object_state/left
~~~~

## Option) Tunning
- You can adjust parameters for the HMM so it will be able to fit better to the training data. There are several adjustable parameters in object_estimator/src/object_estimator/obj_state_classifier.py as followings
~~~~
self.nState       = 30        #number of hidden states in HMM
self.nEmissionDim = 2         #dimension of inputs
self.cov          = 1.0       #diagonal covariance matrix's covariance value
self.scale        = [1.,1.]   #data scale multiplier
~~~~
self.cov (diagonal element value in covariance matrix for observed data) is the parameter you can modify first. This author recommends to adjust it until the Baum Welch training process returns 0~400. 

## Creating a new estimator
### Data collection
- Repeat an object lifting task and rosbag topics. For example, we used a following launch file to record necessary topics from a Baxter robot.
~~~~bash
roslaunch object_estimator record_topic.launch
~~~~
Save data files in a data folder with label names. The folder name should include a label name as a part of it. See a following example that include "heavy" or "light". 
![Alt text](docs/data_folder.png?raw=true "Data folder")
Each folder should contain a set of bag files recorded from the data collection process. The topic should include an end-effector wrench topic with message type, [EndpointState](msg/EndpointState.msg). 

- Set proper rostopic and rosservice names to the launch file.

### Training and Testing
Same as the above



## ETC
### Arguments for the main launch file
Under construction

### Convergence error
If you recieves following message after launching the estimator, it means the current HMM failed to fit to the observed data. You have to tune the HMM's parameter as the above. 
~~~~bash
GHMM ghmm.py:148 - sreestimate.c:ghmm_cmodel_baum_welch(986): NO convergence: log P(8.225583e+03) < log P-old(1.085295e+04)! (n = 2)
~~~~

### Option) Running procedure with the Baxter Experiment Stack
- Running procedure for simulation
~~~~bash
roslaunch baxter_gazebo baxter_world.launch gui:=false
rosrun rviz rviz
rosrun baxter_tools enable_robot.py -e && rosrun baxter_tools tuck_arms.py -u
roslaunch baxter_pipeline_daehyung.launch
roslaunch manipulator_controller action_executive.launch
rosrun manipulator_controller executive.py --r sim 
~~~~

- Test
~~~~bash
rostopic pub /speech_recognition std_msgs/String "data: 'pick up the left most'" 
~~~~
