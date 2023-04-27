<div align="center"><a href="https://unige.it/en/">
<img src="img/genoa_logo.png" width="20%" height="20%" title="University of Genoa" alt="University of Genoa" >
</a></div>

<h1 align="center"> Experimental Robotics Lab Project 2022 </h1> 

>**Author: Omotoye Shamsudeen Adekoya**  
 **Email: adekoyaomotoye@gmail.com** </br>
 **Student ID: 5066348**

## Outline 

1. [Introduction](#intro)
1. [Description of the Software Architecture](#desc)
    * [Component Diagram](#comp-diag)
    * [State Diagram](#state-diag) 
    * [Sequence Diagram](#seq-diag)
2. [Installation and Running Procedure](#install-run)
    * [Installation](#install)
    * [Running](#run)
3. [System Limitations and Possible Improvements](#limits)
4. [Project timeline](#timeline)

> _read the [docs](https://omotoye.github.io/Experimental-Robotics-Project.2022/index.html) for a more comprehensive detail about the [code-base (api)](https://omotoye.github.io/Experimental-Robotics-Project.2022/api.html). You can find everything contained in this readme and more in the docs..._

<a name="intro"></a>

# Introduction 

This project is the first version *(assignment 1)* of the _Experimental Robotics Course_ of a [Robotics Engineering](https://corsi.unige.it/en/corsi/10635) Master's degree at the [University of Genoa](https://unige.it/en/). It is a simulation scenario of a surveillance robot being deployed in an indoor environment. The first version covers the creation of the architecture by putting in place some *nodes/components* that don't exactly perform the task that they are specified to carry out but act as a template to be built on in later versions of the project. The architecture also contains some component that does exactly what they are always going to be required to do in the entire lifetime of the architecture. 

**The Surveillance Scenario is as follows;**

The robot is supposed to survey an environment with rooms connected by corridor(s). The robot would move around the corridor until there's a **reachable** room that needs to be **urgently** visited _(i.e. the room has not been visited/surveyed for a while)_, when such a room exists, the robot navigates to that room and survey's the room for a set amount of time and then leaves back to the corridor. if there's no reachable urgent room available after a while of surveying a corridor, the robot moves to any other available corridor if it exists, otherwise, it keeps on surveying the current corridor. This sequence is continued infinitely until the battery of the robot goes low. If the robot battery goes low at any point in this sequence, whatever action the robot had planned to do is preempted and the robot navigates to the charging stations, it remains there until the battery becomes full and then it goes back to surveying the area. There's an ontology node that reasons about things like what rooms are reachable when a room needs to urgently be visited... basically facts about the surveillance scenario. There's also another node that manages the internal state of the robot like the battery life. All of these nodes/components would be described more extensively in the next section. For more information about the specific requirements of the first version of this project *(assignment 1)*, consult the [assignment specification](docs/assignment-specification.pdf) pdf document. 


<a name="desc"></a>

## Description of the Software Architecture 

<a name="comp-diag"></a>

### Component Diagram

<img src="docs/uml_diagram/exprob_uml.png"  title="Component Diagram Version 1" alt="Component Diagram Version 1" >

<a name="state-diag"></a>

### State Diagram

<a name="seq-diag"></a>

### Sequence Diagram

<a name="install-run"></a>

## Installation and Running Procedure

<a name="install"></a>

### Installation 

<a name="run"></a>

### Running 

<a name="limits"></a>

## System Limitations and Possible Improvements

<a name="timeline"></a>

## Project Timeline 

### 25/01/2023
 - [x] Start of the project, initialize the GitHub Repo
### 02/02/2023
 - [x] Install all the required software and packages on my WSLg. 
### 03/02/2023 - 05/02/2023
 - [x] Study the class material and watch some class recordings. 
 - [x] Understand and Develop a plan for the Project 1 of the course. 
### 07/02/2023
 - [x] Make a paper sketch of the intended architecture. 
 - [x] Make a paper sketch of the states and transitions in the state machine
 - [x] Define the structure of the state machine with smach. 
 - [x] Test the state machine by putting a dummy task in each of the states. 
### 09/02/2023
 - [x] Define the *messages/services/actions* required for the interfaces in the architecture. 
 - [x] Create a server node to take instructions from the state machine and robot controller and then interface with aRMOR service.
### 10/02/2023
 - [x] Complete all the nodes required for Phase 1 of the surveillance senario. 
 - [x] Test the architecture to see that phase 1 works as intended.  
### 11/02/2023
 - [x] Create a control package for the robot state node and robot controller node. 
 - [x] Create a navigation package for the dummy navigation node. 
### 12/02/2023 - 19/02/2023
 - [x] Integrate the controller, robot-state, knowledge and navigation with the state machine based on properly defined messages for interfacing. 
 - [x] Complete all the nodes and logic required for the Phase 2 of the surveillance senario.
 - [x] Complete all the nodes and logic required for the Phase 3 of the surveillance senario. 
### 20/02/2023 - 22/02/2023
 - [x] Test/Fix bugs/Refactor the architecture to see that all the nodes are working together as intended for the surveillance senario.
### 22/02/2023 - 23/02/2023
 - [x] Refactor the code base for clarity and readability
 - [x] Add proper commenting to the code base. 
### 23/02/2023 - 24/02/2023
 - [x] Create a UML diagram of the proposed architecture. 
 - [x] Create the State Diagram. 
### 25/04/2023 - 27/02/2023
 - [ ] Add a Sphinx documentation to the repo 
 - [ ] Create the Sequence diagram
 - [ ] Complete the readme as per assignment requirements. 

_**End of version-1.0.0 of the Project...**_

