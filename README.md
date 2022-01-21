# Overview

This is an rolling horizon extension of OpenRidepoolSimulator (developed by Matthew Zalesak and Vindula Jayawardana). The editted parts are followings. 
- Edit buffer to consider future requests. (buffer.cpp)
- Add waiting logic when a vehicle arrives at the request location before it comes in (simulator.cpp)
