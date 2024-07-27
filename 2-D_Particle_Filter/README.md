# Particle Filter

Rahul Gondi, Venkata Guddeti

## Introduction

Particle filter is a Monte Carlo algorithm used to solve statistical inference problems. In this project, the turtle location and heading direction in maze was infered using particle filter. The green turtle is the actual location while the orange turtule is the estimated location. The arrows are particles. Blue arrows stand for low probability particles while red arrows stand for high probability particles. There are four sensors installed on the front, back, left and right of the turtle. The sensors measure its perpendicular distances to the closest walls in four directions, possibly bounded by some sensor limit. 


![](figures/particle_filter_start.png)  |  ![](figures/particle_filter_converged.png) 


## References

[Beacon Based Particle Filter](https://github.com/mjl/particle_filter_demo)

Some of the code in this project were revised from the Beacon Based Particle Filter project.


## Notes

I found that the number of particles is definitely the most important parameters for particle filter. However, it slows down the computations.

If the number of particles is not sufficiently large, sometimes adjusting the standard deivation of Gaussian kernel might make it converges faster.


