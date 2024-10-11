# Research Documentation

## Problem 1:
Develop a scene using the GUI.

Steps:
1. Go through the [isaac sim tutorials](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html).

## Problem 2: 
Develop a scene using scriping.

I have developed a series of diagrams to help formulate the best approach to solving this. Refer to the [simulation documenation](./simDocumentation.md) for more details about high level architecture. I have mostly noticed that there are two approaches to solving screating a scene in Isaac sim which is using the Isaac sim API or the omniverse API. THe isaac sim API seems to focus on using a world object as the container for the objects relevant ot the thing that you are trying to simulate.  The world object will hold a scene, observations, dataLoggers, and tasks. This can be used to construct a scene that is dynamic but still contained under one object. The omni API is more low level and it takes a more procedural programming focus which means that calls must be made using the USD python API. The omniverse API is more performant and seems to load a lot faster. 
