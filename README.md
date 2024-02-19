# Jenkins - ROS2 Integration 

This repository focuses on establishing a continuous integration pipeline using Jenkins, streamlining the process of triggering CI workflows whenever a pull request is made to a remote repository.

The Jenkins pipeline coordinates a sequence of vital tasks. Initially, it gains access to the Git repository. Subsequently, it proceeds to execute `docker build` to construct the image, followed by execution of `docker compose to run the container. Within the Dockerfile, the pipeline executes ROS2 Google Tests to verify the functionality of the waypoint server. This comprehensive sequence forms the lifecycle of our CI process.

## Checklist of Installation

- ROS2 (Robot Operating System)
- Jenkins
- Docker

## Usage

1\. Start Jenkins Server.

If there is no run_jenkins.sh in the repository:
```
wget https://raw.githubusercontent.com/TheConstructAi/jenkins_demo/master/run_jenkins.sh && bash run_jenkins.sh
```
If run_jenkins.sh exists in the repository:

```
bash run_jenkins.sh
```
To get the jenkins_address
```
jenkins_address
```

2\. Using the jenkins_address, login using the credentials:

```
admin: admin
```
You sholud see the CP24_ROS2_CI when you login 

<img width="1222" alt="image" src="https://github.com/leokim0711092/Checkpoint24_ROS2_CI/assets/106298370/4c336d2d-4ce1-40a0-86e3-4c4093c8f404">


3\. Now let's trigger the build through a pull request:

```
1. Add a new file to this repository and ask to merge.
2. Once the request to merge is accepted, monitor the CP24-ROS2 pipeline and the platform's display.
```

## Output

View consile output in Jenkins to check result
<img width="261" alt="image" src="https://github.com/leokim0711092/Checkpoint24_ROS1_CI/assets/106298370/3f5f084d-a290-40eb-9d44-7caad9ab7e91">

After the execution is complete, you will be able to see the below outputs:
<img width="485" alt="image" src="https://github.com/leokim0711092/Checkpoint24_ROS2_CI/assets/106298370/15eb490a-5494-4c0a-93ea-5e1a72019e43">


<img width="1259" alt="image" src="https://github.com/leokim0711092/Checkpoint24_ROS2_CI/assets/106298370/3907700c-e92e-4ca1-9678-52b48452c612">

