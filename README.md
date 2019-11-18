# CARLA-DeepDriving
Implementing [DeepDriving][dd-url] with [CARLA simulator][carla-url].  


## Background
**DeepDriving**:

DeepDriving shows that by extracting certain information (i.e. affordance indicators) using CNN from an image taken by a typical RGB dash cam, one will be able to control the vehicle in highway traffic with speed adjusting and lane changing ability.

DeepDriving proposed [13 affordance indicators][avs-fig] in 2 systems:  
   ```
   On Marking: angle, toMarking_(L|M|R), dist_(L|R)
   In Lane:    angle, toMarking_(LL|ML|MR|RR), dist_(LL|MM|RR)  
   ```

The collaboration between two systems are critical during lane changing. To make smooth transition, DeepDriving makes an overlapping area when both systems are active. However, narrowing down the vast amount of information available to the driving agent to only 13 affordance indicators is a bit unrealistic, even only on highway. Some notable assumptions that DeepDriving made are:  
+ Only (1|2|3) lane configuration
+ Ego vehicle is faster than the traffic
 
Despite the somewhat narrow scope, DeepDriving still demonstrates some interesting potentials. Also, since the primary goal of the project is just to implement the algorithm from DeepDriving using a more realistic simulator, CARLA, the inadequacy is more of a reminder than a concern especially when we implement the controling logic.


**CARLA**:

CARLA is an open urban driving simulator focused on supporting the development autonomous driving systems. Various measurements (e.g. the location of the car, the width of the lane, etc.) are readily available during simulation thanks to its convenient [PythonAPI][carla-py-url] and fully annotated maps. Also, various sensors and cameras (e.g. RGB camera, depth camera, lidar, etc.) are available. Some other nice features are configurable (vehicle|map|weather), synchronous mode and no-rendering mode. The synchronous mode turns out to be critical to record the data in the way we want.


## Data Collection

[comment]: # (I am not sure if I should write "how to use the code" or "how did I implement this" kind of documentation. Also, I need to update the usage once cli flag is supported)

The data being collected during the simulation are:
1. Pictures taken by the dash cam
2. Ground truth affordance indications associated with each picture

To start the simulation, execute  `<carla_dir>/CarlaUE4.sh Town04 --benchmark -fps=10`  
 
Since DeepDriving is based on highway, [Town04][town04-url] is being used. Also, since Town04 incluedes some non-highway road, during the data collection, once the ego vehicle found not on the highway, the frames and groundtruth will not be recorded. Note it is possible that after some time the vehicle will be on highway and its frame will be recorded once it is on the highway so you might notice some discontinuity in the collected frames. 

To start generating data, execute `src/data/generate_data.py`

All the parameters such as the number of ego vehicles, NPCs, the simulation time limit, etc. can be configured through cli. The only required arguments are `duration` and `name` of the simulation and if one argument is missing it will be provided with one tested default value that should work.

Simplest case (300 seconds simulation called exp):  
```bash
python3 generate_data.py --duration 300 --name exp 
```

Slightly more interesting (300 seconds simulation called exp with 5 ego cars and 100 NPC cars):  
```bash
python3 generate_data.py --duration 300 --name exp --ego-cars 5 --npc-cars 100
```

The cameras' angle, location (wrt to the ego vehicle), orientation and resolution are fully configurable:  
```bash
python3 generate_data.py --duration 300 --name exp --resolution-x 200 --resolution-y 100 --cam-yaw 90 --cam-pitch 10 --cam-z 1.4 --fov 115
```

To debug with live affordance indicators print on console:  
```bash
python3 generate_data.py --duration 300 --name exp --debug
```

When the simulation ends, you get (e.g. for 5 ego vehicles):

```bash
data/{name}/
├── {name}_labels.csv
├── v0
├── v1
├── v2
├── v3
└── v4
```

While the `{name}_labels.csv` has the following header:

```
image-id,angle,toMarking_L,toMarking_M,toMarking_R,dist_L,dist_R,toMarking_LL,toMarking_ML,toMarking_MR,toMarking_RR,dist_LL,dist_MM,dist_RR,velocity(m/s),in_intersection

```

The frame number together with the ego vehicle number and experiment name are used as the unique identifier for the image-id. The `in_intersection` boolean can be used to filter out the images we don't want later in the deep learning stage, according to the assumptions made by DeepDriving.

Once you have enough data and ready to train the neural networks, execute `merge.sh` to merge labels from multiple experiments into a single dataset. Addtionally, you can use `--verbose` flag to see some useful information about the dataset and `--remove-file` flag to have the original labels removed.

[comment]: # (**Details on how the `generate_data.py` script works:** I will add how the code works later, probably in another md file like contributions.md)


## Nerual Network

Jupyter notebooks used for quick exploration are included in `notebook/`. The corresponding python code are included in `src/models/`.

Following [DeepDriving's][dd-url] suggestions, the standard AlexNet is tried. However, due to time constriant, not enough data is collected to effectively evaluate the model. Note that you can check `notebook/train.ipynb` to see some **preliminary** results.


## Reference
+ DeepDriving: [Website][dd-url] | [Paper][dd-paper]  
+ CARLA:       [Website][carla-url] | [Paper][carla-paper]  


[dd-url]: http://deepdriving.cs.princeton.edu/
[dd-paper]: https://arxiv.org/abs/1505.00256/
[carla-url]: http://carla.org/
[carla-paper]: https://arxiv.org/abs/1711.03938/ 
[carla-py-url]: https://arxiv.org/abs/1711.03938://carla.readthedocs.io/en/latest/python_api/ 
[avs-fig]: https://www.ics.uci.edu/~daohangt/img/avs.PNG "Illustration of the affordance representation"
[town04-url]: http://carla.org/2019/01/31/release-0.9.3/
[town04-fig]: https://www.ics.uci.edu/~daohangt/img/town04.PNG "Beautiful Town04 with highway"

## Remark
 This source code is based on results obtained from a project commissioned by the New Energy and Industrial Technology Development Organization (NEDO).
