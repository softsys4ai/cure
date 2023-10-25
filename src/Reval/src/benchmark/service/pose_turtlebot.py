import subprocess

def GetCurrPose():
    subprocess.check_call("python current_pose_turtlebot.py '%s'", shell=True)
def GetCurrPoseAMCL():
    subprocess.check_call("python current_pose_turtlebot_phy.py '%s'", shell=True)    
def PoseSamples():
    subprocess.check_call("python pose_samples_turtlebot.py '%s'", shell=True)  
