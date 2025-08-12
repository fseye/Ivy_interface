import os
from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    return LaunchDescription([
        # lancement de joy node
        Node(
            package= 'joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone':0.05,
                'autorepeat_rate' : 20.0
            }]
        ),

        #lancement du noeu control management

        Node(
            package='control_management_pkg',
            executable='control_management',
            name='control_management',
            output= 'screen'
        ),

        Node(
            package='rembobineur_pkg',
            executable='rembobineur_control',
            name='rembobineur_Control',
            output= 'screen'
        ),

        Node(
            package='stepper_pkg',
            executable='stepper_control',
            name='stepper_node',
            output= 'screen'
        ),

        Node(
            package='nourrisse_pkg',
            executable='nourrice_control',
            name='nourrice_control',
            output= 'screen'
        ),

        Node(
            package='vannes_pkg',
            executable='vannes_control',
            name='vannesControl',
            output= 'screen'
        ),

        Node(
            package='interface_pkg',
            executable='ivy_interface',
            name='interface_node',
            output= 'screen'
        ),

        # test
        

    ])
