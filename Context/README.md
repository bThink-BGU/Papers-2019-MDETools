# Papers-2019-MDETools
Code appendix for the MDETools 2019 paper
https://mdetools.github.io/mdetools19/challengeproblem.html

## What's here

* **README.md**: This file
* **pom.xml**: Maven project file
* **src**: Source code (both Java and Javascript)

## How to Run
* From the commandline:
  1. Make sure [Maven](http://maven.apache.org) and Java are installed and are known to the commandline (e.g. in UNIX, part of the `$PATH`).
  1. Generate a `.jar` file: Run `mvn -P uber-jar package` at the repository's top-level directory.
  1. Run: `java -cp target/RoboCup-Context-0.5-DEV.uber.jar il.ac.bgu.cs.bp.leaderfollower.BPJsRobotControl`
  1. Run the challenge simulation.
  1. Once the simulation is running and you can see the game - press the "start sim" button of the BP program.

## Notes
* The settings file `GameSettings.txt` is stored in the `resources` folder.


## Infrastructure
* This project uses [BPjs](https://github.com/bThink-BGU/BPjs).
* BPjs uses the Mozilla Rhino Javascript engine. See [here](https://developer.mozilla.org/en-US/docs/Mozilla/Projects/Rhino) for project page and source code.
