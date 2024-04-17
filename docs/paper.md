---
sidebar_position: 1
slug: /
---

import ControlTheory from './control-theory.md';
import Sensors from './sensors.md';
import SwerveDrive from './swerve-drive.md';
import Telemetry from './telemetry.md';
import Vision from './vision.md';

# Formal Paper

## Intro
The nature of the FIRST Robotics Competition fuses design and engineering, embracing the pillars of mechanical, electrical, and software engineering. When people join the team with an interest in software engineering, they typically concentrate solely on that field, much like my own initial approach. I thought that stretching myself thin across many aspects of robotics would worsen me as a programmer. I could not have been more wrong. After many years, I’ve now learned from experience just how important understanding the electrical and mechanical function of the robot is as a programmer. If we imagine the code as the robot's brain, it becomes clear that to maximize the movement of its limbs, the robot's brain must first understand its body. This is something that I’d learned, but didn’t come easy, and hasn't yet been emphasized enough in the team. As it stands now, I hold a vast majority of the programming knowledge, and my departure at graduation could throw the team for a loop. This is part of what I aim to solve in my project.

## Research Question
My research question is: “What are the key programming principles in robotics and how can they be effectively taught to the team?” This question can be split up into two parts, the research/experience around understanding many of the important programming principles, and the research required to find the best way to impart this knowledge onto the team. These are both very different types of research, making this an interesting project to pursue.

## Literature Review
A lot of the research for this project, at least the first half, gaining the knowledge and experience, has already been acquired through my many years on the robotics team. A lot of this knowledge I’d gotten from experience or from countless sources scattered across the web. The fact that a lot of this information is either not documented, or hard to find makes it difficult for new programmers to gain in depth knowledge without trial and error. This is another part of my project, consolidating vital information from my experience and around the internet into one cohesive wiki. Moving on, here’s the research I’ve done specific to this project, not only learning new programming concepts, but also learning how these sources convey their information:

- [Controls Engineering in the FIRST Robotics Competition (Graduate-level control theory for high schoolers, by Tyler Veness)](https://file.tavsys.net/control/controls-engineering-in-frc.pdf)  
A very detailed and in-depth paper (300+ pages) explaining one of the most important robotics principles, Control Theory, and all the math behind it. This is the theory behind making mechanical systems function in a controlled, predictable, and repeatable way. This may seem simple in principle, but in reality it quickly becomes very complicated. An intuitive example of this is when balancing a pencil on your finger. First of all, you need your eyes; a way to measure the pencil’s state. This allows you to react when the state is unfavorable. This alone is not enough, since with just this, you’d only move your hand after the pencil starts falling. The real power comes from your brain's ability to predict the future state of the pencil. This is called a “feedforward controller”. Your brain models the system and how it reacts, so it intuitively knows that the pencil will continue to fall when it's tipped to the side. These two principles provide the foundation for control theory and how to actuate complex mechanisms. 

- [2015 FIRST World Championships Conference - Motion Planning & Control in FRC](https://www.youtube.com/watch?v=8319J1BEHwM)  
An hour-long recorded presentation on motion planning and control for FRC. Motion planning is the process of harnessing control theory to create a trajectory for the robot to follow. It can get very complicated, and is very important, especially for the Autonomous Period during the first 15 seconds of the match.

- [Citrus Circuits 2016 Fall Workshops - Motor Sizing](https://www.youtube.com/watch?v=U4pgviiEwLg)  
An hour-long video on how to pick the right size of motor and gearbox for any application and goes in-depth into the math behind motor curves, torque, voltage, and current

- [Whitepaper: Swerve Drive Skew and Second Order Kinematics](https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964)  
A paper on the skewing that is noticeable when translating and rotating simultaneously with swerve drive and the math behind quantizing the drive states to compensate for this.

- [REV Robotics NEO Brushless Motor - Locked-rotor Testing](https://www.revrobotics.com/neo-brushless-motor-locked-rotor-testing/)  
Motor failure and temperature testing for keeping safe current limits in software to prevent burnouts.

- [State Space Physics Simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html)  
How to simulate motors, encoders, gyroscopes, arms, mechanical systems, elevators, and more in a virtual simulated environment. This allows code testing away from the robot, which drastically reduces bugs and improves development time.

- [Swerve Drive Odometry](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html)  
An odometer is a system that measures where a given object is in space. Your car has one to measure how far you’ve driven. For swerve drive, it's a little more complicated, and there is a bunch of math behind fusing all the data regarding speed and direction of each wheel into an accurate position on the field. This explains most of that.

- [State Space Kalman Filters](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html)  
Kalman filters are a way to fuse many different metrics that may be inaccurate, imprecise, or noisy into one very accurate reading. This involves modeling your system and simulating the relationships between the inputs and outputs.

- [Path Planning and Ramsete Controllers](https://docs.wpilib.org/en/stable/docs/software/pathplanning/index.html)  
Path planning is the process of defining a path on the field and getting the robot to follow it as accurately as possible. Because of outside factors, you must use a pose estimator or odometer to account for errors and compensate on the fly. There is also a variety of algorithms for creating a spline from points that preserve acceleration and result in a smooth, optimized motion. The most common algorithm used in this case is Bezier Curves.

- [Pathplanner Library](https://github.com/mjansen4857/pathplanner)  
Pathplanner is an alternative to the built-in WPILib PathWeaver that introduces better support for swerve drive.

- [Docusaurus](https://docusaurus.io/)  
The final platform I’ve decided to use. It’s completely free and open source, looks great, has collaboration built in, and is intended to be published to a website. Exactly what I’ve been looking for all this time.

## Methodology
My research mostly consists of finding what other teams do well and discovering how they do it. I’ve discovered that a lot of teams do well consistently because they document their process and solutions effectively, either writing papers or hosting recorded workshops. I aim to instill those tried and tested methods in the way I write and structure my wiki. 

## Expected Results
At the end of all my research, I should have a complete understanding of how advanced robotics teams handle their software, and how to effectively structure the information in my wiki in a way that is approachable and informative.

## Approach
I plan to spend the next few weeks working on porting over my existing content from Obsidian to Docusaurus, and continue to add more content. Given that almost all my research is done, the majority of my time now will be spent documenting all that I’ve learned. I expect this to take many weeks, and I’ll likely get peer review to see how approachable it is. I’ll also work on the domain name, and site theming. Once that’s done, I’ll start writing my paper, and likely be able to use a lot of the content on my website in my paper as well. All-in-all I’d say I'm on-pace and will be able to complete the project in the given time.

## Part 1
> <ControlTheory />

## Part 2
> <Sensors />

## Part 3
> <SwerveDrive />

## Part 4
> <Telemetry />

## Part 5
> <Vision />