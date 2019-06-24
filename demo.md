## DEMO

### Automatically

Many configurations can be customized (300 seconds simulation called demo-a with 5 ego cars and 300 NPC cars):  
```bash
python3 generate_data.py --duration 300 --name demo-a --ego-cars 5 --npc-cars 300
```

The cameras' angle, location (wrt to the ego vehicle), orientation and resolution are fully configurable:  
```bash
python3 generate_data.py --duration 300 --name demo-b --resolution-x 200 --resolution-y 100 --cam-yaw 90 --cam-pitch 10 --cam-z 1.4 --fov 115
```

### Manually

Since Carla's builtin autopilot function is somewhat limited and if you might want to control the ego vehicle manually.

First do
```bash
python3 manual_control.py --filter <EGO_CAR_TYPE>
```

Then
```bash
python3 generate_data.py --duration 300 --name demo-c --debug 2 --npc-cars 10 --ego-cars 1 --ego-type <EGO_CAR_TYPE>
```
