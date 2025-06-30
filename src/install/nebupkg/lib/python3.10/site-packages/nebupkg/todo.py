import subprocess
import time

def main():
    comandos = [
        #"source /opt/ros/humble/setup.bash && ros2 run nebupkg emociones",
        #"source /opt/ros/humble/setup.bash && ros2 run nebupkg rostros",
        "source /opt/ros/humble/setup.bash && ros2 run nebupkg model_node",
        "source /opt/ros/humble/setup.bash && ros2 run nebupkg voz_node",
    ]

    for cmd in comandos:
        subprocess.run(["gnome-terminal", "--", "bash", "-c", f"{cmd}; exec bash"])


