
---
## 结构化文本说明
### 消息格式
```json
{
    "header": {
        "robot_id": "mobile_dual_arm_robot",
        "msg_type": "check"
    },
    "payload": {
    }
}
```
统一的 JSON 格式，包含 `header` 和 `payload` 两部分。	
- `header` 包含机器人 ID、类型和消息类别	
- `robot_id`：唯一标识每个机器人
	**vacuum_adsorption_robot**：气吸附机器人
	**Magnetic Adsorption Robot**：磁吸附机器人
	**mobile_dual_arm_robot**：AGV+双臂机器人
- `msg_type`：区分消息类型，定义如下：
	**Yes** : 机器人终端设备使能
	**command** : 控制命令
	**response**: 响应
	**query** : 查询
	**status_update**：设备状态上报
- `payload` 包含具体的指令和参数，以及机器人状态信息。

例如：	
- `chassis`: 包含移动指令和参数(仅在msg_type为command时存在)	
- `arms`: 包含每个机械臂的指令和参数(仅在msg_type为command时存在)	
- `robot_info`: 包含机器人状态信息，如电池、电量、传感器数据等(在msg_type为status_update时存在)
---
## 共同约束
1. 所有终端设备需要实现`设备使能`及`响应`，参考 [[ROS2控制平台 - 与server_node通信协议#设备使能]]
---
## 命令示例
### 气吸附机器人(**vacuum_adsorption_robot**)
#### 设备使能
请求：上位机 -> 下位机
```json
{
    "header": {
        "robot_id": "vacuum_adsorption_robot",
        "msg_type": "check"
    },
    "payload": {
        "radar": {
            "command": "check"
        },
        "camera": {
            "command": "check"
        },
        "agv": {
            "command": "check"
        }
    }
}
```
响应：下位机 -> 上位机
```json
{
    "header": {
        "robot_id": "vacuum_adsorption_robot",
        "msg_type": "response"
    },
    "payload": {
        "radar": {
            "command": "check",
            "result":"yes"
        },
        "camera": {
            "command": "check",
            "result":"yes"
        },
        "agv": {
            "command": "check",
            "result":"yes"
        }
    }
}
```

#### 单设备打开/关闭
请求：上位机 -> 下位机
```json
{
    "header": {
        "robot_id": "vacuum_adsorption_robot",
        "msg_type": "check"
    },
    "payload": {
        "radar": {
            "command": "true"
        },
        "camera": {
            "command": "true"
        },
        "agv": {
            "command": "false"
        }
    }
}
```
响应：下位机 -> 上位机
```json
{
    "header": {
        "robot_id": "vacuum_adsorption_robot",
        "msg_type": "response"
    },
    "payload": {
        "radar": {
            "command": "true",
            "result":"yes"
        },
        "camera": {
            "command": "true",
            "result":"yes"
        },
        "agv": {
            "command": "false",
            "result":"yes"
        }
    }
}
```

#### AGV运动控制
##### 急停
请求：上位机 -> 下位机
```json
{
    "header": {
        "robot_id": "agv",
        "msg_type": "command"
    },
    "payload": {
        "agv": {
            "command": "emergency_stop"
        }
    }
}
```

响应：下位机 -> 上位机
```json
{
    "header": {
        "robot_id": "agv",
        "msg_type": "response"
    },
    "payload": {
        "agv": {
            "command": "emergency_stop",
            "result":"yes"
        }
    }
}
```

##### 行驶
请求：上位机 -> 下位机
```json
{
    "header": {
        "robot_id": "agv",
        "msg_type": "command"
    },
    "payload": {
        "agv": {
            "command": "move",
            "data": {
                "vx": 0.0,
                "vy": 0.0,
                "wz": 0.0
            }
        }
    }
}
```
响应：下位机 -> 上位机
```json
{
    "header": {
        "robot_id": "agv",
        "msg_type": "response"
    },
    "payload": {
        "agv": {
            "command": "move",
            "result":"yes"
        }
    }
}
```

#### 多通道超声采集/分析交互
##### 采集
请求：下位机 -> 上位机
```json
{
    "header": {
        "robot_id": "agv",
        "msg_type": "command"
    },
    "payload": {
        "agv": {
            "command": "capture"
        }
    }
}
```
响应：上位机 -> 下位机
```json
{
    "header": {
        "robot_id": "agv",
        "msg_type": "response"
    },
    "payload": {
        "agv": {
            "command": "capture",
            "result":"capture_successed"
        }
    }
}
```
##### 保存
请求：下位机 -> 上位机
```json
{
    "header": {
        "robot_id": "agv",
        "msg_type": "command"
    },
    "payload": {
        "agv": {
            "command": "save"
        }
    }
}
```
响应：上位机 -> 下位机
```json
{
    "header": {
        "robot_id": "agv",
        "msg_type": "response"
    },
    "payload": {
        "agv": {
            "command": "save",
            "result":"save_successed"
        }
    }
}
```
##### 作业完成
请求：下位机 -> 上位机
```json
{
    "header": {
        "robot_id": "agv",
        "msg_type": "command"
    },
    "payload": {
        "agv": {
            "command": "work_complete"
        }
    }
}
```
响应：上位机 -> 下位机
```json
{
    "header": {
        "robot_id": "agv",
        "msg_type": "response"
    },
    "payload": {
        "agv": {
            "command": "work_complete",
            "result":"work_complete_successed"
        }
    }
}
```

##### 闸门通知
请求：上位机 -> 下位机
```json
{
    "header": {
        "robot_id": "agv",
        "msg_type": "update_status"
    },
    "payload": {
        "agv": {
            "status": "alarm_on"
        }
    }
}
```

---

#### 推杆控制
请求： 上位机 -> 下位机
```json
{
    "header": {
        "robot_id": "vacuum_adsorption_robot",
        "msg_type": "command"
    },
    "payload": {
        "agv": {
            "command": "single_scan",
            "ig35_start": 11,
            "scan_speed":12,
            "ig35_end": 0
        }
    }
}
```
响应：下位机 -> 上位机
```json
{
    "header": {
        "robot_id": "vacuum_adsorption_robot",
        "msg_type": "response"
    },
    "payload": {
        "agv": {
            "command": "single_scan",
            "result": "single_scan_successed"
        }
    }
}
```

#### 雷达
##### 位姿
请求：无
响应：上位机 -> 下位机
```json
{
    "header": {
        "robot_id": "agv",
        "msg_type": "response"
    },
    "payload": {
        "lidar": {
            "x": 1.23,
            "y": 4.56,
            "angle": 0.78,
            "target_idx": 2,
            "loc_stat": 1,
            "state": "running"
        }
    }
}
```

##### 路径
请求： 上位机 -> 下位机
```json
{
    "header": {
        "robot_id": "vacuum_adsorption_robot",
        "msg_type": "command"
    },
    "payload": {
        "lidar": {
            "command": "nav_path",
            "target_x": 1100,
            "target_y": 2600,
            "push_accuracy": 0.58999999999999997,
            "nav_accuracy": 0.20000000000000001,
            "scan_speed":12,
            "ig35_start": 11,
            "ig35_end": 0
        }
    }
}
```
响应：下位机 -> 上位机
```json
{
    "header": {
        "robot_id": "agv",
        "msg_type": "response"
    },
    "payload": {
        "lidar": {
            "command": "nav_path",
            "path": [
                {
                    "x": 1.0,
                    "y": 2.0,
                    "theta": 0.5,
                    "type": "normal"
                },
                {
                    "x": 1.5,
                    "y": 2.5,
                    "theta": 0.6,
                    "type": "target"
                }
            ]
        }
    }
}
```
---
### AGV+双臂机器人(mobile_dual_arm_robot)
#### 设备使能
同 [[ROS2控制平台 - 与server_node通信协议#设备使能]]

#### 单设备打开/关闭
同[[ROS2控制平台 - 与server_node通信协议#单设备打开/关闭]]

---
#### 机械臂控制
##### 笛卡尔坐标系-机械臂位姿上报
响应：下位机 -> 上位机
```json
{
    "header": {
        "msg_type": "status_update",
        "robot_id": "mobile_dual_arm_robot"
    },
    "payload": {
        "arms": [
            {
                "arm_id": "left_arm",
                "pos_type": "cartesian",
                "rx": -2.2865527549505038,
                "ry": 0.68890498868481043,
                "rz": -1.0114384277361006,
                "x": -0.050695879814473647,
                "y": 0.72263686057726506,
                "z": 0.45536359751856992
            },
            {
                "arm_id": "right_arm",
                "pos_type": "cartesian",
                "rx": 0.0,
                "ry": 0.0,
                "rz": 0.0,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            }
        ]
    }
}
```

---
##### 关节坐标系-机械臂位姿上报
响应：下位机 -> 上位机
```json
{
    "header": {
        "msg_type": "status_update",
        "robot_id": "mobile_dual_arm_robot"
    },
    "payload": {
        "arms": [
            {
                "arm_id": "left_arm",
                "pos_type": "joint",
                "joint1": 0.94740714746500909,
                "joint2": 1.531040098778365,
                "joint3": -0.25133460282212666,
                "joint4": 1.188376833850991,
                "joint5": -1.162305631938384,
                "joint6": -0.52049106634325815
            },
            {
                "arm_id": "right_arm",
                "pos_type": "joint",
                "joint1": 0.0,
                "joint2": 0.0,
                "joint3": 0.0,
                "joint4": 0.0,
                "joint5": 0.0,
                "joint6": 0.0
            }
        ]
    }
}
```

---
#### 机械臂控制

---
##### 笛卡尔坐标系-机械臂控制
###### 直线规划
请求：上位机 -> 下位机
```json
{
    "header": {
        "robot_id": "mobile_dual_arm_robot",
        "msg_type": "command"
    },
    "payload": {
        "arms": [
            {
                "arm_id": "left_arm",
                "command": "cartesian_move",
                "parameters": {
                    "x": 0.2,
                    "y": 0.1,
                    "z": 0.3,
                    "roll": 0.0,
                    "pitch": 0.5,
                    "yaw": 0.0,
                    "speed": 45
                }
            },
            {
                "arm_id": "right_arm",
                "command": "cartesian_move",
                "parameters": {
                    "x": -0.2,
                    "y": 0.1,
                    "z": 0.25,
                    "roll": 0.0,
                    "pitch": -0.5,
                    "yaw": 0.0,
                    "speed": 45
                }
            }
        ]
    }
}
```

响应：下位机 -> 上位机
```json
{
    "header": {
        "robot_id": "mobile_dual_arm_robot",
        "msg_type": "response"
    },
    "payload": {
        "arms": [
            {
                "arm_id": "left_arm",
                "command": "cartesian_move",
                "result": "cartesian_move_successed"
            },
            {
                "arm_id": "right_arm",
                "command": "cartesian_move",
                "result": "cartesian_move_successed"
            }
        ]
    }
}
```

###### 圆弧规划
请求：上位机 -> 下位机
```json

```

响应：下位机 -> 上位机
```json

```

##### 关节坐标系-机械臂控制
请求：上位机 -> 下位机
```json
{
    "header": {
        "robot_id": "mobile_dual_arm_robot",
        "msg_type": "command"
    },
    "payload": {
        "arms": [
            {
                "arm_id": "left_arm",
                "command": "joint_move",
                "parameters": {
                    "val1": 0.2,
                    "val2": 0.1,
                    "val3": 0.3,
                    "val4": 0.0,
                    "val5": 0.5,
                    "val6": 0.0,
                    "speed": 45
                }
            },
            {
                "arm_id": "right_arm",
                "command": "joint_move",
                "parameters": {
                    "val1": -0.2,
                    "val2": 0.1,
                    "val3": 0.25,
                    "val4": 0.0,
                    "val5": -0.5,
                    "val6": 0.0,
                    "speed": 45
                }
            }
        ]
    }
}
```

响应：下位机 -> 上位机
```json
{
    "header": {
        "robot_id": "mobile_dual_arm_robot",
        "msg_type": "response"
    },
    "payload": {
        "arms": [
            {
                "arm_id": "left_arm",
                "command": "joint_move",
                "result": "joint_move_successed"
            },
            {
                "arm_id": "right_arm",
                "command": "joint_move",
                "result": "joint_move_successed"
            }
        ]
    }
}
```

##### 工件坐标系-机械臂控制
同[[ROS2控制平台 - 与server_node通信协议#笛卡尔坐标系-机械臂控制]]

---
#### AGV控制
同 [[ROS2控制平台 - 与server_node通信协议#行驶]]

--- 

#### 图片像素坐标交互
请求：上位机 -> 下位机
```json
{
    "header": {
        "robot_id": "vacuum_adsorption_robot",
        "msg_type": "command"
    },
    "payload": {
        "camera": {
            "command": "image_pos",
            "x": 100,
            "y": 200,
            "scan_mode":"Long",
            "spacing":10,
            "shrink_factor":12,
            "default_InterPoints":5
        }
    }
}
```

响应：下位机 -> 上位机
```json
{
    "header": {
        "robot_id": "vacuum_adsorption_robot",
        "msg_type": "command"
    },
    "payload": {
        "camera": {
            "command": "image_pos",
            "result":"image_pos_successed"
        }
    }
}
```

---
### 视频流
``` json
{
  "header": {
    "robot_id": "AGV_ARM",
    "msg_type": "status_sensor"
  },
  "payload": {
    "camera_url": "rtsp://192.168.1.100:554/live"
  }
}
```


### 涡流波形图
```json
{
    "header": {
        "robot_id": "mobile_dual_arm_robot",
        "msg_type": "command"
    },
    "payload": {
        "eddy_current": [
            {
                "channel_index": 1,
                "data": 233
            },
            {
                "channel_index": 2,
                "data": 1212
            }
        ]
    }
}
```

---
# 感控扫查
## 涡流感控扫查-Start
上位机 -> ROS2 (Request):
```JSON
{
    "header": {
        "robot_id": "mobile_dual_arm_robot",
        "msg_type": "command"
    },
    "payload": {
        "eddy_current": {
            "command": "start"
        }
    }
}
```

ROS2 -> 上位机 (Response):
```JSON
{
    "header": {
        "robot_id": "mobile_dual_arm_robot",
        "msg_type": "command"
    },
    "payload": {
        "eddy_current": {
            "command": "start",
            "result": "start_succeed"
        }
    }
}
```

## 涡流感控扫查-Stop

上位机 -> ROS2 (Request):
```JSON
{
    "header": {
        "robot_id": "mobile_dual_arm_robot",
        "msg_type": "command"
    },
    "payload": {
        "eddy_current": {
            "command": "stop"
        }
    }
}
```

ROS2 -> 上位机 (Response):
```JSON
{
    "header": {
        "robot_id": "mobile_dual_arm_robot",
        "msg_type": "command"
    },
    "payload": {
        "eddy_current": {
            "command": "stop",
            "result": "stop_succeed"
        }
    }
}
```

## 涡流感控扫查-点云


上位机 -> ROS2:
```JSON
{
    "header": {
        "robot_id": "mobile_dual_arm_robot",
        "msg_type": "query"
    },
    "payload": {
        "eddy_current": {
            "command": "cloud_point"
        }
    }
}
```

ROS2 -> 上位机:
```JSON
{
    "header": {
        "robot_id": "mobile_dual_arm_robot",
        "msg_type": "query"
    },
    "payload": {
        "eddy_current": {
            "command": "cloud_point",
            "result": "AABARw...[此处为约6.6MB的Base64字符串]..."
        }
    }
}
```

---
# robot.js(临时文件)
```json
{
    "header": {
        "robot_id": "AGV_ARM",
        "msg_type": "command"
    },
    "payload": {
        "chassis": {
            "command": "move",
            "parameters": {
                "x": 0.1,
                "y": 0.0,
                "yaw": 0.8,
                "speed": 0.5
            }
        },
        "arms": [
            {
                "arm_id": "left_arm",
                "command": "cartesian_move",
                "parameters": {
                    "x": 0.2,
                    "y": 0.1,
                    "z": 0.3,
                    "roll": 0.0,
                    "pitch": 0.5,
                    "yaw": 0.0
                }
            },
            {
                "arm_id": "right_arm",
                "command": "cartesian_move",
                "parameters": {
                    "x": -0.2,
                    "y": 0.1,
                    "z": 0.25,
                    "roll": 0.0,
                    "pitch": -0.5,
                    "yaw": 0.0
                }
            }
        ],
        "robot_info": {
            "battery": true,
            "lidar": true,
            "camera": true,
            "arm1_position": true,
            "arm2_position": true,
            "agv_position": true
        }
    }
}
```