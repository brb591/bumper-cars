# Autonomous Bumper Cars

## Introduction

We propose to bring bumper cars into the 21st century by simulating the carnival game with autonomous vehicles in a closed arena.  The challenges include friend-or-foe determination, path planning, obstacle avoidance, and score maximization strategies.  Vehicles shall receive points for hitting other vehicles and lose points for being hit.  Head-to-head collisions won't score.  The best case outcome will include the ability for humans to challenge the AIs.

## Project Goal

### Basic Goal

The bumper car arena shall be rectangular with rounded corners to prevent cars from getting stuck, and the arena shall contain 2 or more bumper cars that start at random locations and headings.  Each game lasts for a predetermined length of time.  The arena shall contain various obstacles (polls, boxes, etc.) to make the game more interesting.

In the simplest case:

* All cars are a team of one.  Each car attempts to maximize its score, and the car with the highest score wins.
* At any moment, a given car is in the "attack" state or the "defend" state.  (TODO - Criteria for selecting state?)
  * attack - The car drives directly towards its target.  (TODO - What if the target goes behind an obstacle?)
  * defend - The car drives 45 degrees to either side of the ray cast from the attacker through the defender.  This maximizes the the rate of change of distance.

### Extended Goals

Resources permiting, we might attempt to implement some or all of these additional functionalities:

- [ ] [Target motion](https://en.wikipedia.org/wiki/Target_Motion_Analysis) or [target angle](https://en.wikipedia.org/wiki/Target_angle) analysis to determine foe position and velocity.  This enables intercepting the foe at its future position rather than just pointing its current position.
- [ ] Human interaction - Allow people to play against the bots.
- [ ] Teams with individual maximization - Allow for the organizaiton of cars into 2 or more teams.  Cars should not attack members of their own team.  Each car tries to maximize its score, and the team with the highest aggregate score wins.
- [ ] Teams with team maximization - As above, but cars coordinate to maximize the team score rather than their individual score.
- [ ] Testing with physical agents (JetBots, etc.)
- [ ] Reinforcement learning to create AIs that execute the given strategy with maximum effect

## Level of Autonomy

The bumper cars will operate under level 5 autonomy.  Opportunities for deployment include:

* Existing bumper car facilities with few patrons - The game is more fun if more than a few players are involved.
* Target motion analysis (TMA), if implemented, has obvious military applications.  Submarines, for example, rely on TMA to determine a firing solution without divulging own-ship's position.  Aerial vehicles could do the same.

## Description

This project is most likely to be purely based on simulation.  Each bumper car will have 3 or more cameras to provide 360 degrees of vision coverage.  Cars shall not have any other sensors.  This provides for the easiest transition to physical agents, should that opportunity arise.

Agents shall process the camera inputs using neural networks and use ROS for path planning.

## Related Work

-- Mention any projects that inspired this project; do research about other current solutions that can help you

## Team Organization

* Brian Bauer
* David Kalbfleisch

## Software and Development Tools

* GitHub - to coordinate development
* Gazebo - simulation
* Blender & Gimp - to construct 3D models
* ROS - planning, reinforcement learning, and other AI logic

## Milestones

- [ ] 15 March 2021: 3D arena and car models created
- [ ] 22 March 2021: Robots can identify other vehicles and plot course to avoid all other vehicles
- [ ] 5 April 2021: Robots can predict movement of other vehicles and incorporate expected movement into avoidance plan
- [ ] 19 April 2021: Some of the robots may be deployed with the desire to hit other vehicles
- [ ] 3 May 2021: Presentation of results

## References

* [Gazebo - ROS 2 integration overview](http://gazebosim.org/tutorials?tut=ros2_overview)
