# 🤖 Unity ROS2 ML Robotics Navigation

A Unity project combining **ROS2 Navigation**, **SLAM**, and **ML-Agents Reinforcement Learning** for Turtlebot3 robot simulation.

![Unity](https://img.shields.io/badge/Unity-2022.3-blue)
![ML-Agents](https://img.shields.io/badge/ML--Agents-0.28.0-green)
![ROS2](https://img.shields.io/badge/ROS2-Humble-orange)

## 📋 Overview

This project extends the [Unity Robotics Nav2-SLAM Example](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example) with **reinforcement learning capabilities**. A Turtlebot3 robot learns to navigate to targets in a warehouse environment using Unity ML-Agents.

### Features

- 🏭 **Warehouse Environment**: Randomized warehouse with shelves, boxes, and obstacles
- 🤖 **Dual Robot Setup**: 
  - `robot1`: ML-Agents controlled (learns via PPO)
  - `robot2`: ROS2/Nav2 controlled (classical navigation)
- 🧠 **Reinforcement Learning**: PPO algorithm learns go-to-goal behavior
- 🔗 **ROS2 Integration**: Works with Nav2 and SLAM via ROS-TCP-Connector

## 🏗️ Project Structure

```
Robotics-Nav2-SLAM-Example/
├── Nav2SLAMExampleProject/          # Unity Project
│   ├── Assets/
│   │   ├── Scripts/
│   │   │   ├── AGVController.cs     # Differential drive controller
│   │   │   ├── LaserScanSensor.cs   # LiDAR simulation
│   │   │   └── ...
│   │   ├── turtlebot3/
│   │   │   ├── TurtlebotExplore.cs  # ML-Agents Agent script
│   │   │   └── ...
│   │   └── Scenes/
│   │       └── RLWarehouseScene.unity
│   └── ...
├── ros2_docker/                      # ROS2 Docker setup
└── configs/                          # ML-Agents training configs
    └── turtlebot_explore.yaml
```

## 🚀 Quick Start

### Prerequisites

- Unity 2022.3.x
- Python 3.8 with ML-Agents 0.28.0
- (Optional) Docker for ROS2/Nav2

### Training the Agent

1. **Set up Python environment**:
   ```powershell
   cd C:\UnityRoboticsProjects
   python -m venv mlagents38-env
   .\mlagents38-env\Scripts\Activate
   pip install mlagents==0.28.0 torch==1.8.1+cpu -f https://download.pytorch.org/whl/torch_stable.html
   ```

2. **Start training**:
   ```powershell
   mlagents-learn configs\turtlebot_explore.yaml --run-id=my_training
   ```

3. **Run Unity**:
   - Open `Nav2SLAMExampleProject` in Unity
   - Open `RLWarehouseScene`
   - Press **Play**

### Using a Trained Model

1. Drag your `.onnx` file into Unity Assets
2. Select `robot1` in Hierarchy
3. In **Behavior Parameters** → **Model**, assign your ONNX file
4. Set **Behavior Type** to `Inference Only`
5. Press Play

## 🧠 How the Agent Learns

### Observations (7 floats)
| # | Observation | Range | Description |
|---|-------------|-------|-------------|
| 1 | Forward to target | [-1, 1] | Is target ahead (+1) or behind (-1)? |
| 2 | Right to target | [-1, 1] | Is target right (+1) or left (-1)? |
| 3 | Distance | [0, 1] | Normalized distance to target |
| 4 | Angle to target | [-1, 1] | How much to turn to face target |
| 5 | Robot X position | [-1, 1] | Position relative to boundary |
| 6 | Robot Z position | [-1, 1] | Position relative to boundary |
| 7 | Angular velocity | [-1, 1] | Current rotation speed |

### Actions (2 continuous)
| # | Action | Range | Description |
|---|--------|-------|-------------|
| 1 | Forward/Back | [-1, 1] | Linear velocity command |
| 2 | Turn | [-1, 1] | Angular velocity command |

### Rewards
| Event | Reward | Purpose |
|-------|--------|---------|
| Each step | -0.0005 | Encourage efficiency |
| Getting closer | +0.1 × distance_delta | Encourage progress |
| Facing target while moving | +0.001 | Encourage alignment |
| Collision | -0.02 | Avoid obstacles |
| **Reaching target** | **+2.0** | Main goal |
| Out of bounds | -1.0 | Stay in area |

## 📊 Training Configuration

```yaml
behaviors:
  TurtlebotExplore:
    trainer_type: ppo
    network_settings:
      normalize: true
      hidden_units: 256
      num_layers: 2
    hyperparameters:
      batch_size: 2048
      buffer_size: 20480
      learning_rate: 3.0e-4
    max_steps: 2000000
```

## 🎮 Control Modes

The `AGVController` supports three modes:

| Mode | Description |
|------|-------------|
| `Keyboard` | Manual control with W/A/S/D keys |
| `ROS` | Commands from ROS2 `/cmd_vel` topic |
| `ML` | Commands from ML-Agents neural network |

## 📁 Key Scripts

### `TurtlebotExplore.cs`
The ML-Agents Agent that:
- Collects observations about robot and target
- Receives actions from neural network
- Calculates rewards for training
- Handles episode resets

### `AGVController.cs`
The differential drive controller that:
- Converts velocity commands to wheel speeds
- Supports Keyboard, ROS, and ML control modes
- Handles robot physics via ArticulationBody

## 🔧 Customization

### Change reward weights
Edit the Inspector values on `robot1` → `TurtlebotExplore`:
- `Time Step Penalty`
- `Progress Reward Scale`
- `Collision Penalty`
- `Target Reached Reward`

### Add obstacle avoidance
The LiDAR sensor (`LaserScanSensor.cs`) is available but not yet integrated into observations. Future work could add laser scan data to observations.

## 📚 References

- [Unity ML-Agents Documentation](https://unity-technologies.github.io/ml-agents/)
- [Original Nav2-SLAM Example](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

## 📄 License

Based on [Unity Robotics Nav2-SLAM Example](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example) - Apache 2.0 License

---

Made with ❤️ using Unity, ML-Agents, and ROS2

