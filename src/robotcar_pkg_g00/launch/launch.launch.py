from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Definicion del nodo (pkg_name, node_name)
    rplidar_node = Node(
                    package = 'rplidar_ros',
                    executable = 'rplidar_composition',
                    output = 'screen',
                    parameters = [{
                        'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                        'frame_id': 'laser_frame',
                        'angle_compensate': True,
                        'scan_mode': 'Standard'
                    }])
    pwm = Node(
            package='robotcar_pkg_g00',
            executable='bm_controller_g01'
            )

    laser = Node(
                package='robotcar_pkg_g00',
                executable='laser_node'
                )

    pid_wf = Node(
            package='robotcar_pkg_g00',
            executable='pidWF_node'
            )

    pid_ftg = Node(
            package='robotcar_pkg_g00',
            executable='pidFTG_node'
            )

    follow_the_gap = Node(
            package='robotcar_pkg_g00',
            executable='FTG_node'
            )

    stop = Node(
            package='robotcar_pkg_g00',
            executable='stop_node'
            )

    #retorno "generate_launch_description", separar por comas, el objeto de cada nodo
    return LaunchDescription([
        rplidar_node,
        pwm,
        stop,
        pid_wf,
        pid_ftg,
        follow_the_gap,
        laser
    ])
