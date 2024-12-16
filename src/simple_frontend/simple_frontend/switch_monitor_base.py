from abc import ABCMeta, abstractmethod
from os.path import expanduser
from pathlib import Path
import subprocess
from dataclasses import dataclass


@dataclass
class RemoteHost:
    ip: str
    user: str


class SwitchMonitorBase(metaclass=ABCMeta):
    @abstractmethod
    def __init__(self):
        pass

    def __generate_ssh_command(self, ip: str, user: str, cmd: str):
        # suppress prompt to trust hosts for the first connection
        ssh_option = '-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null'

        # The key should be registered during setup
        ssh_key = Path(expanduser('~'))/'.ssh'/'drs_rsa'

        return f'ssh {ssh_option} -i {ssh_key} {user}@{ip} {cmd}'

    def restart_drs_launch_services(self):
        """
        Very rough implementation to restart drs_launch.service on each ECU
        """
        # The command should be registerd in /etc/sudoers.d/ so that the user can execute it
        # without password
        restart_cmd = 'sudo systemctl restart drs_launch.service'

        targets = [
            RemoteHost(ip='192.168.20.2', user='nvidia'),  # ECU#1
            RemoteHost(ip='192.168.20.1', user='nvidia'),  # ECU#0
        ]

        for t in targets:
            cmd = self.__generate_ssh_command(t.ip, t.user, restart_cmd)
            subprocess.Popen(cmd, shell=True)  # execute in background

    def restart_drs_rosbag_record_services(self):
        """
        Very rough implementation to restart drs_rosbag_record.service on each ECU
        """
        # The command should be registerd in /etc/sudoers.d/ so that the user can execute it
        # without password
        restart_cmd = 'sudo systemctl restart drs_rosbag_record.service'

        targets = [
            RemoteHost(ip='192.168.20.2', user='nvidia'),  # ECU#1
            RemoteHost(ip='192.168.20.1', user='nvidia'),  # ECU#0
        ]

        for t in targets:
            cmd = self.__generate_ssh_command(t.ip, t.user, restart_cmd)
            subprocess.Popen(cmd, shell=True)  # execute in background

    def shutdown_drs_components(self):
        """
        Very rough implementation to shutdown each ECU components gently
        """
        # The command should be registerd in /etc/sudoers.d/ so that the user can execute it
        # without password
        poweroff_cmd = 'sudo poweroff'

        targets = [
            RemoteHost(ip='192.168.20.2', user='nvidia'),  # ECU#1
            RemoteHost(ip='192.168.10.100', user='root'),  # NAS
            RemoteHost(ip='192.168.10.1', user='nvidia'),  # ECU#0, this entry have to come at the very last!
        ]

        for t in targets:
            cmd = self.__generate_ssh_command(t.ip, t.user, poweroff_cmd)
            subprocess.run(cmd, shell=True, capture_output=False)
