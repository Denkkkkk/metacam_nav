
from distutils.core import setup
 
setup(
    version='0.0.0',
    scripts=['scripts/docker_receive_cloud.py','scripts/docker_receive_odom.py','scripts/docker_send.py','scripts/host_receive_cmd_vel.py','scripts/host_send.py'],
    # packages=['mymodels'],
    # package_dir={'': 'scripts'}
)