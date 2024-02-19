# Jenkins - ROS2 Integration 

This repository focuses on establishing a continuous integration pipeline using Jenkins, streamlining the process of triggering CI workflows whenever a pull request is made to a remote repository.

The Jenkins pipeline coordinates a sequence of vital tasks. Initially, it gains access to the Git repository. Subsequently, it proceeds to execute `docker build` to construct the image, followed by execution of `docker compose to run the container. Within the Dockerfile, the pipeline executes ROS1 Unit Tests to verify the functionality of the waypoint server. This comprehensive sequence forms the lifecycle of our CI process.

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

<img width="1352" alt="image" src="https://github.com/leokim0711092/Checkpoint24_ROS1_CI/assets/106298370/8c7c9ae0-c152-4992-8a52-f4227d62d933">


3\. Now let's trigger the build through a pull request:

```
1. Add a new file to this repository and ask to merge.
2. Once the request to merge is accepted, monitor the CP24-ROS2 pipeline and the platform's display.
```

## Output

View consile output in Jenkins to check result
<img width="261" alt="image" src="https://github.com/leokim0711092/Checkpoint24_ROS1_CI/assets/106298370/3f5f084d-a290-40eb-9d44-7caad9ab7e91">

After the execution is complete, you will be able to see the below outputs:

<img width="649" alt="image" src="https://github.com/leokim0711092/Checkpoint24_ROS1_CI/assets/106298370/25720cd1-b630-4db4-bedd-70a20e0fb00c">

<img width="613" alt="image" src="https://github.com/leokim0711092/Checkpoint24_ROS1_CI/assets/106298370/f70fbf11-0ac8-4bc9-963c-ad82569f9314">

