import subprocess

def GetCurrPose():
    subprocess.check_call("python current_pose_husky.py '%s'", shell=True)
def PoseSamples():
    subprocess.check_call("python pose_samples_husky.py '%s'", shell=True)  

GetCurrPose()
#PoseSamples()    
