#!/usr/bin/env python3
import math
import paramiko
import json
import struct
import time
from registers import *
import numpy as np
build_path = '/root/TD-KFM_redpitaya/build/'



def rp_get_register(ssh, reg):
    '''Readout one RedPitaya register via ssh.'''
    command = build_path+'parameter_readout 0 '+str(reg)
    stdin, stdout, stderr = ssh.exec_command(command, timeout=1)
    return int(stdout.readlines()[0])


def rp_set_register(ssh, reg, value):
    '''Write to one RedPitaya register via ssh.'''
    command = build_path+'parameter_readout 1 '+str(reg)+' '+str(value)
    stdin, stdout, stderr = ssh.exec_command(command, timeout=1)
    stdout.readlines()

def rp_get_multireg_float(ssh, param_index):
    '''Executes command to readout float value that is split onto multiple registers.'''
    command = build_path+'parameter_readout 2 '+str(param_index)
    stdin, stdout, stderr = ssh.exec_command(command, timeout=1)
    return float(stdout.readlines()[0])

def rp_set_multireg_float(ssh, param_index, f_value):
    '''Executes command to split float value onto multiple registers.'''
    command = '/root/TD-KFM_redpitaya/build/parameter_readout 3 '+str(f_value)+' '+str(param_index)
    stdin, stdout, stderr = ssh.exec_command(command, timeout=1)
    stdout.readlines()

def rp_get_register_bit(ssh, reg, bit):
    r = rp_get_register(ssh, reg)
    return r & (1 << int(bit))

def rp_set_register_bit(ssh, reg, bit, status):
    r = rp_get_register(ssh, reg)
    if status: r = r | (1 << int(bit))
    else: r = r & ~(1 << int(bit))
    rp_set_register(ssh, reg, r)

def trigger_parameter_update(ssh, param_index):
    """Sends the update index to the device and flips the update-request bit."""
    rp_set_register(ssh, REG_PARAMS_INDEX, param_index)
    rp_set_register_bit(ssh, REG_FLAGS, FLAGS_UPDATERQ_BIT, 1)

def trigger_parameter_readout(ssh, param_index):
    """Sends the requested index to the device and flips the read-request bit."""
    rp_set_register(ssh, REG_PARAMS_INDEX, param_index)
    rp_set_register_bit(ssh, REG_FLAGS, FLAGS_READRQ_BIT, 1)



def main():
    with open('../config/ssh_info.json') as f:
        ssh_info = json.load(f)

    ssh = paramiko.SSHClient()
    ssh.load_system_host_keys()
    ssh.connect(ssh_info['HOST'], ssh_info['PORT'], ssh_info['USER'], ssh_info['PASS'])
    print('connected to host: ', ssh_info['HOST'])
    
    td_kfm_controller = TDKFMController(ssh)
    td_kfm_controller.print_status()

    return td_kfm_controller


if __name__ == '__main__':
    td_kfm_controller = main()
    ssh = td_kfm_controller.ssh
