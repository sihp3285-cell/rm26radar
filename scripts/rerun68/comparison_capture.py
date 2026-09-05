#!/usr/bin/env python3
"""Read-only parameter snapshots of each running test, using an isolated ROS domain."""
import json,os,subprocess,time
from comparison_build import CMP
from run import BASE

def main():
    env=os.environ.copy();env.update(ROS_DOMAIN_ID='196',ROS_LOCALHOST_ONLY='1',ROS_HOME=str(CMP/'param_ros'))
    while True:
        progress=json.loads((CMP/'progress.json').read_text())
        if progress.get('status')=='all_recorded':return
        folder=BASE/progress['folder'].split('/')[-1]
        if (folder/'source.json').exists():
            for node,name in [('pose_node','pose_parameters.yaml'),('position_prior_node','prior_parameters.yaml')]:
                dest=folder/name
                if dest.exists() and dest.stat().st_size>100:continue
                result=subprocess.run(['ros2','param','dump','/'+node,'--no-daemon','--timeout','3'],env=env,capture_output=True,text=True,timeout=15)
                if result.returncode==0 and 'ros__parameters:' in result.stdout:
                    dest.write_text(result.stdout);print('CAPTURE',dest,flush=True)
                else:print('RETRY',node,result.stderr[-200:],flush=True)
        time.sleep(3)

if __name__=='__main__':main()
