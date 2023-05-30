#!/usr/bin/env python

from pathlib import Path
from sys import excepthook
import yaml
import rospkg
import rospy as rp
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamSet, ParamSetRequest


if __name__ == "__main__":

    r = rospkg.RosPack()

    config_path = Path(r.get_path("uav_launch")) / "config" / "mavlink" / "fake_gps.yaml"
    with config_path.open('r') as file:
        config = yaml.load(file)
    
    srv = "/mavros/param/set"
    rp.logdebug("Waiting for service: {}".format(srv))
    rp.wait_for_service(srv)
    mavros_param_setter = rp.ServiceProxy(srv, ParamSet)

    for setting in config:
        int_val = 0
        float_val = 0.0
        value = config[setting]
        param_value = ParamValue()
        if isinstance(value, int):
            param_value.integer = value
        elif isinstance(value, float):
            param_value.real = value
        else:
            rp.loginfo("Setting '{}' with value '{}' could not be set as param. \
                        Value needs to be of type 'int' or 'float'.".format(setting, value))
            continue
        
        try:
            req = ParamSetRequest(setting, param_value)
            mavros_param_setter(req)
        except rp.ServiceException as exc:
            rp.loginfo("Service did not process request: {}".format(exc))