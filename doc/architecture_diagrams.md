# 架构图草稿

下面先给出 4 张架构图草稿，聚焦当前仓库实际会用到的链路：Isaac Sim 训练、`src` 下键盘/扩散策略伺服部署、真机模仿学习训练、PushCube。

## 1. Isaac Sim 强化学习框架

```mermaid
flowchart TB
    subgraph assets["机器人资产层"]
        urdf["assets/robot_description<br/>URDF / mesh"]
        cfg["rl_train/config.py<br/>解析关节、限位、初始位姿"]
        resolved["generated/*.resolved.urdf<br/>重写 package:// mesh 路径"]
    end

    subgraph isaac["Isaac Sim 仿真层"]
        app["SimulationApp<br/>headless / viewer"]
        world["World<br/>地面、桌面、方块"]
        importer["rl_train/isaac_import.py<br/>导入 URDF 为 Articulation"]
        env["HumanoidBrickPickEnv<br/>Gymnasium 环境"]
    end

    subgraph rl["PPO 训练层"]
        vecenv["DummyVecEnv / SubprocVecEnv"]
        ppo["Stable-Baselines3 PPO<br/>MlpPolicy"]
        monitor["Monitor<br/>episode info"]
        callbacks["Callbacks<br/>checkpoint / metrics / eval"]
    end

    subgraph outputs["输出层"]
        logs["logs/ppo_runs<br/>csv / tensorboard"]
        ckpt["checkpoints<br/>ppo_humanoid_brick_*"]
        models["models<br/>final / latest / best"]
        eval["eval_policy.py / demo_pick_brick.py"]
    end

    urdf --> cfg --> resolved --> importer
    app --> world
    importer --> world --> env
    env --> monitor --> vecenv --> ppo
    ppo -->|action| env
    env -->|obs / reward / done / info| ppo
    ppo --> callbacks
    callbacks --> logs
    callbacks --> ckpt
    callbacks --> eval
    ppo --> models
```

## 2. `src` 控制部署框架：键盘控制与 Diffusion Servo

```mermaid
flowchart TB
    subgraph model["机器人模型与控制基础"]
        desc["robot_description<br/>URDF / ros2_controllers.yaml"]
        moveitcfg["robot_moveit_config<br/>SRDF / kinematics / move_group"]
        launch["robot_commander/launch/robot_moveit.launch.xml"]
        rsp["robot_state_publisher"]
        cm["controller_manager<br/>ros2_control_node"]
        jsb["joint_state_broadcaster"]
        controllers["JointTrajectoryControllers<br/>right / left / neck / gripper"]
        hw["robot_hardware<br/>SystemInterface"]
        motor["CAN-FD / 电机驱动"]
    end

    subgraph keyboard["键盘遥操作路径"]
        kbd["robot_keyboard_control<br/>joint_control / cartesian_control"]
        commander["robot_commander<br/>Commander + MoveGroupInterface"]
    end

    subgraph servo["MoveIt Servo 路径"]
        servo_launch["robot_servo_control<br/>servo_control.launch.py"]
        right_servo["right_servo/moveit_servo"]
        left_servo["left_servo/moveit_servo"]
        head_servo["head_servo/moveit_servo"]
    end

    subgraph diffusion["Diffusion Policy Servo 路径"]
        ckpt["checkpoints/latest.pt"]
        dp["diffusion_policy_servo_node<br/>obs buffer + DDPM sampling"]
        safety["安全转换<br/>target_position -> delta -> velocity<br/>clip / dry_run / estop"]
    end

    desc --> launch
    moveitcfg --> launch
    launch --> rsp
    launch --> cm
    cm --> jsb
    cm --> controllers
    controllers --> hw --> motor
    jsb -->|/joint_states| kbd
    jsb -->|/joint_states| dp

    kbd -->|/right_joint_command<br/>/left_joint_command<br/>/neck_joint_command<br/>/open_*_gripper| commander
    commander -->|MoveIt planning / execute| controllers

    servo_launch --> right_servo
    servo_launch --> left_servo
    servo_launch --> head_servo
    ckpt --> dp --> safety
    safety -->|/right_servo/moveit_servo/delta_joint_cmds<br/>JointJog| right_servo
    right_servo --> controllers
    left_servo --> controllers
    head_servo --> controllers
```

注：`robot_moveit.launch.xml` 中 `Commander` 节点当前是注释状态；键盘指令链路需要单独启动 `ros2 run robot_commander commander` 或恢复该 launch 节点。

## 3. 真机模仿学习训练框架

```mermaid
flowchart LR
    subgraph real_robot["真机执行与示教"]
        stack["robot_commander<br/>robot_moveit.launch.xml<br/>use_simulation:=false"]
        keyboard["robot_keyboard_control<br/>joint_control"]
        robot["真实机器人<br/>right arm / gripper"]
        joint_states["/joint_states"]
        commands["/right_joint_command<br/>/open_right_gripper"]
        camera["可选相机<br/>/right_wrist_camera/image_raw"]
    end

    subgraph record["在线录制"]
        recorder["robot_imitation_pipeline<br/>demo_recorder_node"]
        control["demo_control<br/>start / stop --success"]
        raw["data/imitation_raw/episode_xxxxxx<br/>robot_state.npy / action.npy<br/>meta.json / success.json"]
    end

    subgraph bag["rosbag 后处理路径"]
        rosbag["data/rosbag/rosbag2_*"]
        process["scripts/process_rosbag_data.py<br/>提取 controller state"]
        processed["data/processed/real_robot/training_episodes<br/>robot_state: 12<br/>action: 6"]
    end

    subgraph train["Diffusion Policy 训练与评估"]
        dataset["ImitationDataset<br/>窗口切片 + 归一化统计"]
        model["DiffusionPolicy<br/>MLP + timestep embedding"]
        ddpm["DDPM 训练<br/>预测噪声 MSE"]
        ckpt["data/checkpoints/diffusion_policy/latest.pt<br/>model_state_dict + stats + config"]
        offline_eval["scripts/eval_diffusion_policy.py<br/>离线误差评估和图表"]
        deploy["robot_servo_control<br/>diffusion_policy_servo_node"]
    end

    stack --> robot
    keyboard --> commands --> stack
    robot --> joint_states
    robot --> camera
    joint_states --> recorder
    commands --> recorder
    camera --> recorder
    control --> recorder --> raw

    rosbag --> process --> processed
    raw --> dataset
    processed --> dataset
    dataset --> model --> ddpm --> ckpt
    ckpt --> offline_eval
    ckpt --> deploy
```

## 4. PushCube 框架

```mermaid
flowchart TB
    subgraph push2d["PushCube2D baseline"]
        env2d["pushcube2d/env.py<br/>7 维 obs / 2 维 action"]
        scripted["scripted_policy.py<br/>几何专家策略"]
        collect2d["collect_data.py / keyboard_collect.py"]
        npz["data/pushcube2d/*.npz<br/>obs / actions / states / success"]
        bc2d["train_bc.py<br/>NumPy MLP BC"]
        eval2d["eval_policy.py<br/>rollout success / distance"]
        viz2d["visualize_* / compare_experiments.py"]
    end

    subgraph pushisaac["Isaac PushCube v0"]
        scene["check_pose_scene.py + manual_ee_teleop.py<br/>Isaac 场景、桌面、方块、目标区、右臂"]
        record["record_manual_pushcube.py<br/>人工示教采集"]
        hdf5["data/pushcube_manual*.hdf5<br/>obs_dim=13 / action_dim=2"]
        inspect["inspect / plot / filter / augment<br/>数据质量与扩增"]
        bc3d["train_pushcube_bc.py<br/>PyTorch MLP BC"]
        model3d["runs/pushcube_bc*/model.pt<br/>obs_mean / obs_std / model"]
        eval3d["eval_pushcube_policy.py<br/>Isaac 闭环评估 / 视频 / rollout hdf5"]
    end

    subgraph common["共同思想"]
        obs["低维状态<br/>pusher/EE、cube、target、relative vector"]
        action["平面动作<br/>dx / dy"]
        metric["指标<br/>success rate / final distance / cube motion"]
    end

    env2d --> scripted --> collect2d --> npz --> bc2d --> eval2d --> viz2d
    scene --> record --> hdf5 --> inspect --> bc3d --> model3d --> eval3d

    obs -.-> env2d
    obs -.-> scene
    action -.-> bc2d
    action -.-> bc3d
    eval2d -.-> metric
    eval3d -.-> metric
```
