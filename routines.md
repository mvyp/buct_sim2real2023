# Routines

In this section we provide instructions on launching individual components used in the baseline solution.

Before proceeding, you should follow instructions in [this section](./README.md#launch-server-and-create-client-container-without-running) to make sure the server is running and the client has been created.

## Cartographer Navigation

1. Launch a ROS node for navigation

   ```shell
   scripts/shell.sh roslaunch carto_navigation navigation.launch
   ```

2. Specify the navigation target by `rviz`:

   ![](./assets/carto-nav.png)

## Detect a Cube

1. Launch a node for detection and grasping:

   ```shell
   scripts/shell.sh roslaunch rmus_solution manipulate.launch
   ```

2. Move the EP [with keyboard](#cartographer-navigation) to the location where digits on the check station are visible.

   ![](./assets/check-front.png)

3. Detect the digits on the check station by calling a ROS service:

   ```shell
   MODE=9
   scripts/shell.sh rosservice call /image/processor_switch_mode "mode: ${MODE}"
   ```

   Note that, the command can be automatically completed by pressing `Tab`.

   After calling the service, the digits on the check station should be detected and visualized on the `image_view` output:

   ![](./assets/check-front-digits.png) 

   > Reference for the service `/image_processor_switch_mode`:
   > 
   > `mode`:
   > > |Value|Specification|
   > > |:-:|:-:|
   > > | `0` | Halt (Default) |
   > > | `1` ~ `5` | Detect digits `1` ~ `5` |
   > > | `6` | Detect `B` |
   > > | `7` | Detect `O` |
   > > | `8` | Detect `X` |
   > > | `9` | Detect three digits on check station |

4. Stop the service by calling

   ```shell
   scripts/shell.sh rosservice call /image_processor_switch_mode "mode: 0"
   ```

## Grasp a Cube

1. Launch a node for detection and grasping:

   ```shell
   scripts/shell.sh roslaunch rmus_solution manipulate.launch
   ```

2. Move the EP [with keyboard](#cartographer-navigation) towards the front face of a cube

   ![](./assets/cube-front.png)

3. Invoke vision service to locate the cube
   
   ```shell
   scripts/shell.sh rosservice call /image_processor_switch_mode "mode: 3"
   ```

   > The parameter `3` corresponds to the digit 3 on the cube to grasp.

   After calling the service, the digits on the cube should be detected and visualized on the `image_view` output:

   ![](./assets/cube-front-digits.png) 

4. Grasp the cube

   ```shell
   MODE=1
   scripts/shell.sh rosservice call /let_manipulater_work "mode: ${MODE}
   call: ''"
   ```
   
   Note that, the command can be automatically completed by pressing `Tab`.

   > Reference for the service `/let_manipulater_work`:
   > 
   > `mode`:
   > > |Value|Specification|
   > > |:-:|:-:|
   > > | `0` | Robot arm reset |
   > > | `1` | Grasp the cube in front of the EP |
   > > | `2` | Place the cube at hand into the check station   |
   > 
   > `call`: Usused, default value is empty string. You can use this parameters in your method. 

5. Move the EP [with keyboard](#cartographer-navigation) to the front of the check station:

   ![](./assets/check-front-2-third.png)
   
   ![](./assets/check-front-2.png)

6. Find the target box to place the grasped cube:

   ```shell
   scripts/shell.sh rosservice call /image_processor_switch_mode "mode: 7"
   ```

   > `6`, `7`, `8` correspond to `B`, `O`, `X` at the checking station

   ![](./assets/check-front-2-digits.png)

7. Place the cube in the box
   
   ```shell
   rosservice call /let_manipulater_work "mode: 2
   call: ''"
   ```

   ![](./assets/check-front-3.png)

   ![](./assets/check-front-3-third.png)

8. Stop the service

   ```shell
   roslaunch rmus_solution start_game.launch
   ```