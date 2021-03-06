# Probability-Based Time-Optimal Motion Planning of Multi-robots Along Specified Paths

PMRMP is based on PVTPlanner
(Optimal Acceleration-Bounded Trajectory Planning in Dynamic Environments Along a Specified Path
Jeff Johnson, Kris Hauser, ICRA 2012)
to slove probability-based time-optimal motion planning of multi-robots along specificied paths.

## Building

Tested on Ubuntu 16.04.

```
mkdir build
cd build
cmake ..
make
```

## Run example instances

### VIP_PB

````
./VIP_PB
````
then use example/view.m to plot results.

### VIP_CBS

````
./VIP_CBS
````
then use example/view.m to plot results.


If you use this code, please cite

@inproceedings{Wang2020ProbabilityBasedTM,

  title={Probability-Based Time-Optimal Motion Planning of Multi-robots Along Specified Paths},
  
  author={Haonan Wang and Biao Hu and Zhengcai Cao},
  
  year={2020}
  
}
