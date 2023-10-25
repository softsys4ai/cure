## Changing configuration options
To change the configuration options, update the `set_config.py` file to your desire configuration values.

Example of the subset of configuration options:
```python
cost_scaling_factor_g = random.randint(1,20) # A scaling factor to apply to cost values during inflation
update_frequency_g = random.randint(4,7)  # The frequency in Hz for the map to be updated.
publish_frequency_g = random.randint(1,4) # The frequency in Hz for the map to be publish display information.
transform_tolerance_g = round(random.uniform(0.2, 2),2) # Specifies the delay in transform (tf) data that is tolerable in seconds.  For example, a transform being 0.2 seconds out-of-date may be tolerable, but a transform being 8 seconds out of date is not.
footprint_padding_g = round(random.uniform(0.01, 0.05),2) # Amount to pad footprint (m).
combination_method_g = random.randint(0,1) # # Changes the behaviour how the obstacle_layer handles incoming data from layers beyond it. Possible values are "Overwrite" (0), "Maximum" (1) and "Nothing" (99).
```

## Custom gazebo environment
You can use any gazebo environment. To use your own gazebo environment you have to place your `<your_env_name>.map` file to `cd Reval/src/husky_ws/src/husky/husky_gazebo/worlds/` folder. And update the launch file located at `cd Reval/src/husky_ws/src/husky/husky_gazebo/launch/playpen.launch`
Currently we do not support automatic generated environment, which will be in your future release.
```launch
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    # Update Scenario1.world to your world file name
    <arg name="world_name" value="$(find husky_gazebo)/worlds/Scenario1.world"/>
    
    # Do not change
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>
</launch>
```

## Define mission specifications
To define target locations update the coordinates in the `goals.py` file. It is important to define the `narrow_spaces` for your custom gazebo environment to get the `RNS` metric value. Currently the RNS metric only supports maximum 5 target locations, we plan to add more flexibility in our future release.
```python
# goals -> mission specification
narrow_spaces = 5
target_locations = 5

goal1_x = 4.243134021759033
goal1_y = 0.11776113510131836
goal1_z = 0.004264891722451529
goal1_w = 0.9999909053079412

goal2_x = 6.797357082366943
goal2_y = 0.17836666107177734
goal2_z = 0.005639492443043434
goal2_w = 0.9999840979360546

goal3_x = 9.390673637390137
goal3_y = 0.16084671020507812
goal3_z = 0.00666542118821814
goal3_w = 0.9999777858334572

goal4_x = 11.79060173034668
goal4_y = 0.18810081481933594
goal4_z = 0.009660472769838802
goal4_w = 0.9999533365441925

goal5_x = 14.576149940490723
goal5_y = 0.23428869247436523
goal5_z = 0.006430144347818834
goal5_w = 0.9999793264081344
```
