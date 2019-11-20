## Questions:

(A is more confident than a)

+ No information about cars behind the ego-vehicle?  
   A: "simply because the system cannot see things behind. To solve this problem, we make an assumption that the host car is faster than the trafﬁc. Therefore if sufﬁcient time has passed since its overtaking (indicated by a timer), it is safe to change to that lane." -- DeepDrive Paper

+ Distance only in forward direction wrt ego vehicle? (no y/z?)  
   a: I guess it depends on how they implement the control logic
   
+ How can two system be activated at the same time -> duplicate name for same value?  
   A: It is duplicate
   
+ How to handle inactive training images for NN? NN outputs av even if no sense? (e.g. A picture without any car ahead of ego but has/not ground truth label disturbs the NN?)  
   A: Make the angle to determine precedings cars the same as the camera angle (wide)

+ Why not crop out the upper part of the images (i.e. sky)?  
   A: When the front car is very close it will partially cover the upper part. When this happens it might be hard for NN to determine toMarking distance

## Note:
#### DeepDrive:
+ Manual driving to collect data seems necessary to get more diversed data. 
+ "Car perception module is reliable up to 30 meters away" -- DeepDrive Paper 
+ DeepDrive assume only 1/2/3 lane scenario
+ Training Sample Size: 484,815
   
#### Carla:
+ To run simulation faster:  
   Fixed timestep  
   Low quality rendering  
+ No-rendering mode should be useful when visualizing our result but it cannont be used to collect data (Note that in this mode, cameras and other GPU-based sensors return empty data.)  
    
    ```python
    settings = world.get_settings()
    settings.no_rendering_mode = True
    world.apply_settings(settings)
    ```
    
+ Physics-engine of vehicles can be changed (e.g. mass;max_rpm) (PyAPI)
+ Batch setting available that could change multiple cars at the same time
+ Weather can be changed continuously:
   
   ```python
   weather = carla.WeatherParameters(
   cloudyness=80.0,
   precipitation=30.0,
   sun_altitude_angle=70.0)
   world.set_weather(weather)
   ```
   
+ Traffic light can be controlled (PyAPI)  
+ It is difficult to sync using global time/fps small delta might get measurement in last frame -> so use sync mode  
   [A nice explanation about what motivates sync mode and how carla update frame internally](https://github.com/carla-simulator/carla/issues/1274#issuecomment-465567495)
+ My failed attempt to do sync without sync mode provided py carla:

```python
while TIME < MAX_TIME:
    # Not able to sync camera and avs
    avsss = vx_reporter.report(vx,mapp,actor_list) 
    avss = avsss[1]
   # avss.insert(0,FRAME_NUMBER)
   # print(111,FRAME_NUMBER)
    avss = [x if x != None else -1 for x in avss]
    avss.insert(0,"{:.2f}".format(TIME))
    print(TIME)
    print(avsss[0]) 
    print(avss)
    avs_writer.writerow(avss)
    
    time.sleep(INTERVAL)
    TIME += INTERVAL
```


## TODO:
+ Use fixed timestep
+ Figure out how DeepDrive use only 3 scenearios to do and see if we can generalize it  
   inactive systems' output  
   controlling logic
+ Improve simulation speed
+ No rendering speed for visualization
+ Refactoring the code to add cli flags support
