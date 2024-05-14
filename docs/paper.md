---
sidebar_position: 1
slug: /
full_width: true
title: Capstone Paper
---

import ControlTheory from './control-theory.md';
import Sensors from './sensors.md';
import SwerveDrive from './swerve-drive.md';
import Telemetry from './telemetry.md';
import Simulation from './simulation.md';

# <div style={{ display: "none" }}>Capstone Paper</div>

<style jsx>
{`
  .mla-title-page {
    height: calc(100vh - 150px);; /* Full height of the viewport */
    width: 100%;
    display: flex;
    flex-direction: column;
    justify-content: center;
    text-align: center;
    border-bottom: 1px solid #000;
    margin-bottom: 20px;
  }

  .mla-title-page h1,
  .mla-title-page h2,
  .mla-title-page p {
    margin: 0;
  }
`}
</style>

<div className="mla-title-page">
    <h1>Distilling Advanced Programming <br /> Principles For FRC</h1>
    <h2>A Capstone Paper</h2>
    <p>Written by Logan MacAskill</p>
</div>

## Intro
The FIRST Robotics Competition is the ultimate robotics showdown for high-school students worldwide that throws them headfirst into the deep end of STEM. Since 1989, this competition has been the breeding ground for innovation, collaboration, and real-world problem-solving in a form similar to a sport, but for robotics.

Programming is one of the core skills applied in FRC, it gives these robots life and smarts, but only as smart as the programmer who coded it. With robotics programming comes many advanced and important principles that aren't seen elsewhere, and are game-changing in the face of FRC. Through this project, I aimed to dive deeper into some of these principles, and conceive a way to solidify this knowledge for future seasons to come.

## Research Question
It simply comes down to: “What are the key programming principles in robotics and how can they be effectively taught to the team?” This question has two parts, the research/experience for understanding many of these important and advanced programming principles, and the research into finding the best way to impart this knowledge onto the team.

## Sources
I already know a lot about robotics, specifically software, but for this project, I wanted to dive deeper and explore the most advanced and powerful techniques to learn and bestow upon the team. In my research, I found that a lot of the existing literature is hard to find, and quite scattered. This requires a deep understanding of the topic and prerequisite knowledge to understand the literature. This is a hard barrier for entry for a lot of newer students, and a lot of programmers end up falling back to trial and error to learn these difficult concepts. For this project, I searched the web for the best sources of information, and have consolidated it into one cohesive wiki. Here are some of my most important sources:

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
Going about this project, the method of effectively passing down this knowledge to the robotics team must be determined. Initially, teaching a class comes to mind. By leading a course to actively teach members of the team through workshops and hand-on experiences, these important techniques and practices could be learned in a short time. This has a major problem though: legacy. By teaching only a single class, the impact this project has for years to come would be limited. Unless others are willing to continue the course, the knowledge would cease to spread after my graduation.

A practical solution to this problem is imparting this knowledge on a lasting accessible resource, like a website. This way those who want to learn many years later can reference a simple website and learn easily. An easy way to blend teaching with a website is to host a public set of slides, and record a walkthrough of them all. This works for simple topics, but falls short when experienced team members want to single out a single topic. They would have to flip through all the slides or scrub through hours of video just to find the topic that interests them. I spent some time exploring [SliDev](https://sli.dev), but eventually pivoted for this reason.

Finally, the tried and true approach: a wiki. One of the biggest sources of information on the internet is Wikipedia, and their layout allows for easy editing and easy navigation for those who are looking for specific topics. One of the most popular ways to structure a wiki-style information base is with [Obsidian](https://obsidian.md). Obsidian is a markdown style information-base tool that looks great and feature rich. Markdown in particular is very powerful for wikis because it's a way to format plain-text into readable and beautiful websites. This is possible from simple rules that allow for special formatting inside a standard .md files, allowing for quickly creating content without spending time on formatting and other trivial problems.

Obsidian is great, however it lacks support for hosting on a custom website for free. This aspect is crucial to the product as it needs to live somewhere accessible and stay there indefinitely. Again, I spend a great deal of time creating content using Obsidian, but eventually realized this problem and started looking for alternatives. The final platform I decided to use was [Docusaurus](https://docusaurus.io/). It's like Obsidian, but is designed for websites from the ground up. It's open-source and completely free, so I dove right in. I spend a lot of time customizing it for my needs, changing the layout, formatting, and theme. Most of this is documented on their site, using Docusaurus! Anyway, after I had the website set up, I needed to create the content.

The past few months I had spend doing heaps of research and experimentation of many of the most important software practices for FRC and robotics in general. A lot of this came out of the research needed for Swerve Drive, a project I led over the summer. Swerve Drive combines complicated mechanical systems with software, and requires in-depth knowledge of control theory, sensors, and much more to even get working. This provided a great opportunity to do research and apply my findings, actively helping the team reach new heights.

On top of this research derived from hands-on application, I also dove into many of the resources that other teams have, and learned from them. Not only do some of these teams have really great resources, they also structure it in a convenient way. I embraced this moving forward for how I structure my wiki. Before looking at some of these resources, I was under the impression that having dozens of pages all with very specific knowledge linked to each other was the way to go. That way people can look for the specific thing they want to learn about, but I soon learned that information structured in such a way can be overwhelming. Having dozens of pages to sift through and understand is much harder than having only a few that cover complicated concepts step-by-step. Realizing this, I split my wiki into 6 parts:

1. Control Theory
2. Sensors
3. Swerve Drive
4. Telemetry
5. Simulation

Dividing it this way gives learners a way to single out what they want to learn, but also go through step-by-step all the knowledge at once without backtracking too much.

## Acquired Knowledge & Literature Review

## Part 1
> <ControlTheory />

## Part 2
> <Sensors />

## Part 3
> <SwerveDrive />

## Part 4
> <Telemetry />

## Part 5
> <Simulation />

## Conclusion
Through a series of resources, hands-on experimentation, and research into specialized software practices for FRC, this project delved into the complexities and intricacies of robotics, offering insights into Control Theory, Sensors, Swerve Drive, Telemetry, and Simulation. Each discovery was carefully documented and organized into a comprehensive architecture for optimal retention and accessibility, all housed in a purpose-built wiki using Docusaurus.

I hope this resource becomes a great learning experience for intermediate programmers to take that next step into advanced robotics engineering and make their mark on FRC forever.