# Robot Playing Mastermind

This repository contains code and assets for a two-player (and player-vs-robot) implementation of the Mastermind-like game where a robot physically picks and places colored pieces, and vision + language models help evaluate and/or choose guesses.


## Overview

Robot Playing Mastermind implements two gameplay modes:

**Mode 1 — PVP (Player vs Player)**

1. Player 1 chooses a secret color combination of four game pieces.
2. Player 2 controls the robot to pick colored pieces from boxes and place them into the guess slots.
3. After each placement, Player 1 announces:
    * how many colors are correct (regardless of position), and
    * how many pieces are in the correct positions.
    * The game continues until Player 2 correctly matches all 4 pieces (Player 2 wins), or Player 1 survives 10 rounds (Player 1 wins).

**Mode 2 — PVE (Player vs Environment / Robot)**

1. Player chooses a secret color combination of four game pieces.
2. The robot makes guesses (color + position) driven by analysis from the ChatGPT API.
3. A camera system (YOLO-based detector) inspects the robot’s placement and determines whether the guess is correct; the system feeds the results back for the next decision.


## Features

* Physical robot arm control for picking & placing colored pieces.
* Human-in-the-loop gameplay (PVP).
* Autonomous robot guessing using a ChatGPT-powered strategy (PVE).
* Visual verification using a YOLO object-detection model and camera.

Repository structure (suggested)
```graphsql
/ (root)
├─ README.md
├─ src/
│  ├─ robot_control/        # robot control code & SDK wrappers
│  ├─ vision/               # YOLO model, detection scripts, camera interface
│  ├─ llm/                  # ChatGPT API interface & decision logic
│  ├─ ui/                   # control scripts / CLI / GUI
│  └─ utils/
├─ models/                  # trained vision model weights
├─ assets/                  # diagrams, photos, sample configs
└─ docs/                    # setup, wiring diagrams, experiment notes
```


## How to run
Step1. login using github.

Step2. Build docker:
```bash
docker run --rm -it   -v "$PWD:/root/workspaces" -e HOME=/root/ -e DISPLAY=$DISPLAY --net=host --gpus all --name mastermind   finalproject:latest
```


## License 
This project is released under the MIT License — see LICENSE for full text.

