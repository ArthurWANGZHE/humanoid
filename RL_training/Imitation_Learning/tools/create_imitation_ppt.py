#!/usr/bin/env python3
from __future__ import annotations

import zipfile
from pathlib import Path
from xml.sax.saxutils import escape


OUT = Path(__file__).resolve().parents[1] / "humanoid_imitation_learning_report.pptx"
EMU_W = 12192000
EMU_H = 6858000


COLORS = {
    "bg": "F7F9FC",
    "ink": "1F2937",
    "muted": "64748B",
    "blue": "2563EB",
    "cyan": "0891B2",
    "green": "16A34A",
    "orange": "EA580C",
    "purple": "7C3AED",
    "red": "DC2626",
    "line": "CBD5E1",
    "white": "FFFFFF",
}


def emu(x: float) -> int:
    return int(x * 914400)


def xml_text(text: str) -> str:
    return escape(text).replace("\n", "<a:br/>")


def run(text: str, size: int = 24, color: str = "ink", bold: bool = False) -> str:
    b = '<a:b/>' if bold else ''
    return (
        f'<a:r><a:rPr lang="zh-CN" sz="{size * 100}" dirty="0">'
        f'{b}<a:solidFill><a:srgbClr val="{COLORS[color]}"/></a:solidFill>'
        '<a:latin typeface="Microsoft YaHei"/><a:ea typeface="Microsoft YaHei"/>'
        f'</a:rPr><a:t>{xml_text(text)}</a:t></a:r>'
    )


def paragraph(text: str, size: int = 24, color: str = "ink", bold: bool = False) -> str:
    return f'<a:p>{run(text, size=size, color=color, bold=bold)}<a:endParaRPr lang="zh-CN"/></a:p>'


def text_box(shape_id: int, x: float, y: float, w: float, h: float, paras: list[str]) -> str:
    return (
        f'<p:sp><p:nvSpPr><p:cNvPr id="{shape_id}" name="TextBox {shape_id}"/>'
        '<p:cNvSpPr txBox="1"/><p:nvPr/></p:nvSpPr>'
        f'<p:spPr><a:xfrm><a:off x="{emu(x)}" y="{emu(y)}"/><a:ext cx="{emu(w)}" cy="{emu(h)}"/>'
        '</a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom><a:noFill/><a:ln><a:noFill/></a:ln></p:spPr>'
        '<p:txBody><a:bodyPr wrap="square" lIns="0" tIns="0" rIns="0" bIns="0"/>'
        '<a:lstStyle/>'
        f'{"".join(paras)}'
        '</p:txBody></p:sp>'
    )


def rect(
    shape_id: int,
    x: float,
    y: float,
    w: float,
    h: float,
    fill: str,
    line: str = "line",
    radius: bool = True,
    text: str | None = None,
    text_size: int = 20,
    text_color: str = "ink",
    bold: bool = False,
) -> str:
    prst = "roundRect" if radius else "rect"
    tx = ""
    if text is not None:
        tx = (
            '<p:txBody><a:bodyPr wrap="square" anchor="mid" lIns="91440" tIns="45720" rIns="91440" bIns="45720"/>'
            '<a:lstStyle/>'
            f'<a:p><a:pPr algn="ctr"/>{run(text, size=text_size, color=text_color, bold=bold)}</a:p>'
            '</p:txBody>'
        )
    return (
        f'<p:sp><p:nvSpPr><p:cNvPr id="{shape_id}" name="Box {shape_id}"/>'
        '<p:cNvSpPr/><p:nvPr/></p:nvSpPr>'
        f'<p:spPr><a:xfrm><a:off x="{emu(x)}" y="{emu(y)}"/><a:ext cx="{emu(w)}" cy="{emu(h)}"/>'
        f'</a:xfrm><a:prstGeom prst="{prst}"><a:avLst/></a:prstGeom>'
        f'<a:solidFill><a:srgbClr val="{COLORS[fill]}"/></a:solidFill>'
        f'<a:ln w="12700"><a:solidFill><a:srgbClr val="{COLORS[line]}"/></a:solidFill></a:ln>'
        f'</p:spPr>{tx}</p:sp>'
    )


def line(shape_id: int, x1: float, y1: float, x2: float, y2: float, color: str = "line") -> str:
    x = min(x1, x2)
    y = min(y1, y2)
    w = abs(x2 - x1)
    h = abs(y2 - y1)
    flip_h = ' flipH="1"' if x2 < x1 else ""
    flip_v = ' flipV="1"' if y2 < y1 else ""
    return (
        f'<p:cxnSp><p:nvCxnSpPr><p:cNvPr id="{shape_id}" name="Line {shape_id}"/>'
        '<p:cNvCxnSpPr/><p:nvPr/></p:nvCxnSpPr>'
        f'<p:spPr><a:xfrm{flip_h}{flip_v}><a:off x="{emu(x)}" y="{emu(y)}"/>'
        f'<a:ext cx="{emu(w)}" cy="{emu(h)}"/></a:xfrm>'
        '<a:prstGeom prst="straightConnector1"><a:avLst/></a:prstGeom>'
        f'<a:ln w="25400"><a:solidFill><a:srgbClr val="{COLORS[color]}"/></a:solidFill>'
        '<a:tailEnd type="none"/><a:headEnd type="triangle"/></a:ln></p:spPr></p:cxnSp>'
    )


def title_shapes(title: str, subtitle: str | None = None) -> tuple[list[str], int]:
    shapes = [
        rect(2, 0, 0, 13.333, 0.22, "blue", "blue", radius=False),
        text_box(3, 0.65, 0.45, 12.0, 0.55, [paragraph(title, 28, "ink", True)]),
    ]
    next_id = 4
    if subtitle:
        shapes.append(text_box(4, 0.68, 0.98, 11.8, 0.35, [paragraph(subtitle, 14, "muted")]))
        next_id = 5
    return shapes, next_id


def slide_xml(shapes: list[str]) -> str:
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<p:sld xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" '
        'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" '
        'xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">'
        '<p:cSld><p:spTree>'
        '<p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>'
        f'<p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="{EMU_W}" cy="{EMU_H}"/>'
        f'<a:chOff x="0" y="0"/><a:chExt cx="{EMU_W}" cy="{EMU_H}"/></a:xfrm></p:grpSpPr>'
        + rect(1000, 0, 0, 13.333, 7.5, "bg", "bg", radius=False)
        + "".join(shapes)
        + '</p:spTree></p:cSld><p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr></p:sld>'
    )


def bullet_list(items: list[str], size: int = 18) -> list[str]:
    paras = []
    for item in items:
        paras.append(
            '<a:p><a:pPr marL="228600" indent="-171450"><a:buChar char="•"/></a:pPr>'
            f'{run(item, size=size)}<a:endParaRPr lang="zh-CN"/></a:p>'
        )
    return paras


def build_slides() -> list[str]:
    slides: list[str] = []

    shapes = [
        rect(2, 0, 0, 13.333, 7.5, "bg", "bg", radius=False),
        rect(3, 0, 0, 13.333, 0.28, "blue", "blue", radius=False),
        text_box(4, 0.75, 1.25, 11.5, 0.85, [paragraph("人形机器人模仿学习数据录制与真机策略推理", 30, "ink", True)]),
        text_box(5, 0.78, 2.12, 10.5, 0.5, [paragraph("从示范采集、行为克隆训练，到 ROS 2 真机部署的完整链路", 17, "muted")]),
        rect(6, 0.78, 3.15, 3.0, 1.0, "white", text="数据录制", text_size=20, bold=True),
        rect(7, 4.0, 3.15, 3.0, 1.0, "white", text="模型训练", text_size=20, bold=True),
        rect(8, 7.22, 3.15, 3.0, 1.0, "white", text="ROS 推理", text_size=20, bold=True),
        rect(9, 10.44, 3.15, 2.15, 1.0, "white", text="真机执行", text_size=20, bold=True),
        line(10, 3.78, 3.65, 4.0, 3.65, "blue"),
        line(11, 7.0, 3.65, 7.22, 3.65, "blue"),
        line(12, 10.22, 3.65, 10.44, 3.65, "blue"),
        text_box(13, 0.82, 6.35, 6.5, 0.35, [paragraph("汇报内容：工程流程、数据格式、模型结构、部署方式、风险与后续计划", 14, "muted")]),
    ]
    slides.append(slide_xml(shapes))

    shapes, sid = title_shapes("1. 课题目标与当前方案", "用人工遥操作示范训练右臂抓取策略，并在 ROS 中实现真机推理")
    shapes += [
        text_box(sid, 0.8, 1.65, 5.7, 4.6, bullet_list([
            "目标：让右臂学习人工示范中的关节运动与夹爪开合",
            "当前阶段：先建立低维状态行为克隆基线，保证工程链路打通",
            "输入：右臂关节位置 6 + 速度 6 + 夹爪状态 1 = 13 维",
            "输出：右臂关节目标 6 + 夹爪命令 1 = 7 维",
            "部署：ROS 2 节点实时订阅 /joint_states 并发布控制命令",
        ])),
        rect(sid + 1, 7.1, 1.75, 4.8, 3.7, "white"),
        text_box(sid + 2, 7.45, 2.08, 4.0, 0.5, [paragraph("行为克隆 BC", 24, "blue", True)]),
        text_box(sid + 3, 7.45, 2.82, 3.9, 1.55, [paragraph("监督学习：让模型输出动作尽量接近示范动作", 18), paragraph("Loss = MSE(pred_action, demo_action)", 16, "muted")]),
    ]
    slides.append(slide_xml(shapes))

    shapes, sid = title_shapes("2. 系统总体流程", "录制、验证、转换、训练、推理形成闭环")
    x0, y, w, h, gap = 0.75, 2.0, 2.0, 0.85, 0.38
    boxes = [("人工遥操作", "orange"), ("Raw Episode", "blue"), ("数据校验", "green"), ("Dataset", "cyan"), ("BC 训练", "purple"), ("ROS 推理", "blue")]
    for i, (txt, col) in enumerate(boxes):
        x = x0 + i * (w + gap)
        shapes.append(rect(sid + i * 2, x, y, w, h, "white", col, text=txt, text_size=17, bold=True))
        if i < len(boxes) - 1:
            shapes.append(line(sid + i * 2 + 1, x + w, y + h / 2, x + w + gap, y + h / 2, col))
    shapes += [
        text_box(sid + 20, 0.95, 3.5, 11.2, 1.2, [paragraph("核心思想：先把真机示范规范化成可训练数据，再用同一套状态/动作定义训练模型，最后用 ROS 节点把模型输出接回真机控制话题。", 20)]),
        rect(sid + 21, 1.1, 5.25, 3.25, 0.75, "white", text="data/imitation_raw", text_size=16),
        rect(sid + 22, 5.0, 5.25, 3.25, 0.75, "white", text="data/imitation_converted", text_size=16),
        rect(sid + 23, 8.9, 5.25, 3.05, 0.75, "white", text="data/imitation_runs", text_size=16),
    ]
    slides.append(slide_xml(shapes))

    shapes, sid = title_shapes("3. 数据录制模块", "demo_recorder 负责同步 joint state、动作命令和腕部相机")
    shapes += [
        text_box(sid, 0.8, 1.55, 5.85, 4.8, bullet_list([
            "启动右腕相机：/right_wrist_camera/image_raw",
            "启动真机控制栈和键盘遥操作",
            "demo_control start / stop 控制 episode 边界",
            "成功示范标记 valid_for_training=true",
            "失败示范保留但不进入训练",
        ])),
        rect(sid + 1, 7.1, 1.55, 4.85, 4.25, "white"),
        text_box(sid + 2, 7.45, 1.9, 4.2, 2.8, [
            paragraph("录制依赖 topic", 22, "blue", True),
            paragraph("/joint_states", 17),
            paragraph("/right_joint_command", 17),
            paragraph("/open_right_gripper", 17),
            paragraph("/right_wrist_camera/image_raw", 17),
        ]),
        text_box(sid + 3, 7.45, 4.8, 4.0, 0.55, [paragraph("缺失输入会写入 meta.json diagnostics", 15, "muted")]),
    ]
    slides.append(slide_xml(shapes))

    shapes, sid = title_shapes("4. Episode 数据格式", "每条示范保存为标准化目录，训练直接读取 robot_state/action")
    shapes += [
        rect(sid, 0.85, 1.55, 5.6, 4.95, "white"),
        text_box(sid + 1, 1.15, 1.85, 5.0, 3.95, [
            paragraph("episode_000001/", 18, "blue", True),
            paragraph("meta.json / success.json", 15),
            paragraph("timestamps.npy", 15),
            paragraph("robot_state.npy  [T, 13]", 15),
            paragraph("action.npy       [T, 7]", 15),
            paragraph("joint_pos.npy / joint_vel.npy", 15),
            paragraph("obs/right_wrist_camera/*.jpg", 15),
        ]),
        rect(sid + 2, 7.05, 1.55, 5.2, 2.1, "white"),
        text_box(sid + 3, 7.35, 1.85, 4.6, 1.2, [paragraph("robot_state[i]", 20, "green", True), paragraph("右臂位置 6 + 速度 6 + 夹爪状态 1", 17)]),
        rect(sid + 4, 7.05, 4.05, 5.2, 2.1, "white"),
        text_box(sid + 5, 7.35, 4.35, 4.6, 1.2, [paragraph("action[i]", 20, "purple", True), paragraph("右臂目标关节 6 + 夹爪命令 1", 17)]),
    ]
    slides.append(slide_xml(shapes))

    shapes, sid = title_shapes("5. 数据质量验证", "先过滤空数据和无效 episode，再进入训练")
    shapes += [
        text_box(sid, 0.8, 1.55, 5.9, 4.6, bullet_list([
            "frames 必须大于 0",
            "robot_state_dim 必须等于 13",
            "action_dim 必须等于 7",
            "图像帧数与 timestamps 对齐",
            "不允许 NaN / Inf",
            "success.json 中 valid_for_training=true",
        ])),
        rect(sid + 1, 7.2, 1.75, 4.65, 3.7, "white"),
        text_box(sid + 2, 7.55, 2.05, 4.0, 2.3, [
            paragraph("已发现的问题", 22, "red", True),
            paragraph("之前 episode_000001 为 0 帧空数据", 17),
            paragraph("校验器已改为 ERROR", 17),
            paragraph("recorder 新增 diagnostics", 17),
        ]),
    ]
    slides.append(slide_xml(shapes))

    shapes, sid = title_shapes("6. 数据集转换", "将多条 raw episode 合并为训练数据集")
    shapes += [
        rect(sid, 0.95, 1.75, 3.3, 0.85, "white", "blue", text="Raw Episodes", text_size=18, bold=True),
        line(sid + 1, 4.25, 2.18, 5.1, 2.18, "blue"),
        rect(sid + 2, 5.1, 1.75, 3.25, 0.85, "white", "green", text="convert_to_hdf5", text_size=18, bold=True),
        line(sid + 3, 8.35, 2.18, 9.2, 2.18, "green"),
        rect(sid + 4, 9.2, 1.75, 3.0, 0.85, "white", "cyan", text="dataset.npz", text_size=18, bold=True),
        text_box(sid + 5, 0.95, 3.35, 5.7, 2.1, bullet_list([
            "obs_state: [N, 13]",
            "actions: [N, 7]",
            "episode_index: 样本来源",
            "episode_ends: episode 边界",
        ], size=18)),
        text_box(sid + 6, 7.1, 3.35, 4.8, 2.1, bullet_list([
            "splits.json 保存训练/验证划分",
            "manifest.json 记录每条 episode 的样本统计",
            "dataset.hdf5 兼容后续扩展",
        ], size=18)),
    ]
    slides.append(slide_xml(shapes))

    shapes, sid = title_shapes("7. 行为克隆模型", "当前基线采用状态到动作的 MLP")
    shapes += [
        rect(sid, 1.0, 2.0, 2.2, 0.85, "white", "green", text="13 维状态", text_size=18, bold=True),
        line(sid + 1, 3.2, 2.42, 4.3, 2.42, "green"),
        rect(sid + 2, 4.3, 1.65, 3.25, 1.55, "white", "blue", text="MLP\n3 × 256 + ReLU", text_size=18, bold=True),
        line(sid + 3, 7.55, 2.42, 8.65, 2.42, "blue"),
        rect(sid + 4, 8.65, 2.0, 2.2, 0.85, "white", "purple", text="7 维动作", text_size=18, bold=True),
        text_box(sid + 5, 0.95, 4.05, 5.4, 1.5, [paragraph("训练目标：最小化模型预测动作与人工示范动作之间的 MSE。", 20)]),
        text_box(sid + 6, 7.0, 4.05, 4.8, 1.5, [paragraph("优化器：AdamW；默认 batch=128，epoch=50，自动选择 CPU/CUDA。", 20)]),
    ]
    slides.append(slide_xml(shapes))

    shapes, sid = title_shapes("8. 训练产物与评估指标", "以 best validation loss 选择部署 checkpoint")
    shapes += [
        text_box(sid, 0.8, 1.55, 5.9, 4.9, bullet_list([
            "训练命令：ros2 run robot_imitation_pipeline train_bc",
            "输出目录：data/imitation_runs/bc_state",
            "best.pt：验证集 loss 最低的模型",
            "last.pt：最后一个 epoch 的模型",
            "checkpoint 保存 input_dim、action_dim、config、epoch、val_loss",
        ])),
        rect(sid + 1, 7.15, 1.75, 4.7, 3.85, "white"),
        text_box(sid + 2, 7.5, 2.05, 4.0, 2.6, [
            paragraph("汇报建议统计", 22, "blue", True),
            paragraph("episode 数量", 17),
            paragraph("总样本数 N", 17),
            paragraph("训练/验证划分", 17),
            paragraph("best val_loss", 17),
            paragraph("dry-run / 真机测试结果", 17),
        ]),
    ]
    slides.append(slide_xml(shapes))

    shapes, sid = title_shapes("9. ROS 真机推理部署", "新增 policy_inference.launch.py，可 dry-run 也可发布真机命令")
    shapes += [
        text_box(sid, 0.8, 1.45, 5.9, 4.95, bullet_list([
            "加载 data/imitation_runs/bc_state/best.pt",
            "订阅 /joint_states 构造 13 维输入",
            "MLP 输出右臂目标和夹爪命令",
            "发布 /right_joint_command 与 /open_right_gripper",
            "默认 publish_commands=false，仅打印预测",
        ])),
        rect(sid + 1, 7.0, 1.65, 4.95, 4.35, "white"),
        text_box(sid + 2, 7.35, 1.95, 4.3, 2.65, [
            paragraph("安全参数", 22, "green", True),
            paragraph("max_joint_step_rad 限制单步变化", 17),
            paragraph("stale_joint_state_sec 超时保护", 17),
            paragraph("publish_commands 显式开关", 17),
            paragraph("inference_rate_hz 控制频率", 17),
        ]),
    ]
    slides.append(slide_xml(shapes))

    shapes, sid = title_shapes("10. 真机运行安全流程", "先 dry-run，再小步长、低风险执行")
    shapes += [
        rect(sid, 0.9, 1.65, 2.6, 0.75, "white", "blue", text="1. 启动控制栈", text_size=16, bold=True),
        rect(sid + 1, 3.85, 1.65, 2.6, 0.75, "white", "blue", text="2. 检查 joint state", text_size=16, bold=True),
        rect(sid + 2, 6.8, 1.65, 2.6, 0.75, "white", "green", text="3. Policy dry-run", text_size=16, bold=True),
        rect(sid + 3, 9.75, 1.65, 2.6, 0.75, "white", "orange", text="4. 真机发布", text_size=16, bold=True),
        line(sid + 4, 3.5, 2.02, 3.85, 2.02, "blue"),
        line(sid + 5, 6.45, 2.02, 6.8, 2.02, "blue"),
        line(sid + 6, 9.4, 2.02, 9.75, 2.02, "green"),
        text_box(sid + 7, 1.0, 3.25, 11.0, 2.4, bullet_list([
            "机器人处于示范数据覆盖过的初始姿态附近",
            "急停可用，周围无人员和障碍物",
            "第一次执行建议 max_joint_step_rad=0.03~0.08 rad",
            "先观察 dry-run 输出范围，再打开 publish_commands=true",
        ], size=20)),
    ]
    slides.append(slide_xml(shapes))

    shapes, sid = title_shapes("11. 当前状态与后续计划", "已完成工程闭环，下一步提升策略能力")
    shapes += [
        rect(sid, 0.85, 1.55, 5.65, 4.75, "white"),
        text_box(sid + 1, 1.15, 1.9, 5.0, 3.55, [paragraph("已完成", 22, "green", True)] + bullet_list([
            "数据录制与 episode 标准化",
            "空数据检测与 diagnostics",
            "数据集转换与 BC 训练",
            "ROS policy 推理 launch",
            "模仿学习汇报文档与 PPT",
        ], size=17)),
        rect(sid + 2, 6.95, 1.55, 5.45, 4.75, "white"),
        text_box(sid + 3, 7.25, 1.9, 4.8, 3.55, [paragraph("下一步", 22, "blue", True)] + bullet_list([
            "采集更多有效成功示范",
            "记录并汇报 train/val loss",
            "真机 dry-run 与小范围闭环测试",
            "加入视觉输入或时序模型",
            "动作平滑、速度限制与安全策略",
        ], size=17)),
    ]
    slides.append(slide_xml(shapes))

    shapes = [
        rect(2, 0, 0, 13.333, 7.5, "bg", "bg", radius=False),
        rect(3, 0, 0, 13.333, 0.28, "blue", "blue", radius=False),
        text_box(4, 0.95, 1.25, 11.2, 0.8, [paragraph("总结", 34, "ink", True)]),
        text_box(5, 1.0, 2.25, 10.9, 2.7, [paragraph("本阶段已经完成从真机示范录制、数据质量验证、训练数据转换、MLP 行为克隆训练，到 ROS 2 真机策略推理的完整工程链路。", 25)]),
        rect(6, 1.0, 5.35, 3.0, 0.75, "white", "green", text="可继续采数据", text_size=18, bold=True),
        rect(7, 5.15, 5.35, 3.0, 0.75, "white", "blue", text="可训练 baseline", text_size=18, bold=True),
        rect(8, 9.3, 5.35, 3.0, 0.75, "white", "purple", text="可接入 ROS 推理", text_size=18, bold=True),
    ]
    slides.append(slide_xml(shapes))
    return slides


def write_static_parts(z: zipfile.ZipFile, slide_count: int) -> None:
    overrides = [
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>',
        '<Default Extension="xml" ContentType="application/xml"/>',
        '<Override PartName="/ppt/presentation.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.presentation.main+xml"/>',
        '<Override PartName="/ppt/slideMasters/slideMaster1.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slideMaster+xml"/>',
        '<Override PartName="/ppt/slideLayouts/slideLayout1.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slideLayout+xml"/>',
        '<Override PartName="/ppt/theme/theme1.xml" ContentType="application/vnd.openxmlformats-officedocument.theme+xml"/>',
    ]
    overrides += [
        f'<Override PartName="/ppt/slides/slide{i}.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slide+xml"/>'
        for i in range(1, slide_count + 1)
    ]
    z.writestr("[Content_Types].xml", f'<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">{"".join(overrides)}</Types>')
    z.writestr("_rels/.rels", '<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="ppt/presentation.xml"/></Relationships>')

    sld_ids = "".join([f'<p:sldId id="{255+i}" r:id="rId{i}"/>' for i in range(1, slide_count + 1)])
    z.writestr("ppt/presentation.xml", f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:presentation xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
<p:sldMasterIdLst><p:sldMasterId id="2147483648" r:id="rId{slide_count+1}"/></p:sldMasterIdLst>
<p:sldIdLst>{sld_ids}</p:sldIdLst>
<p:sldSz cx="{EMU_W}" cy="{EMU_H}" type="wide"/><p:notesSz cx="6858000" cy="9144000"/>
</p:presentation>''')
    rels = [
        f'<Relationship Id="rId{i}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slide" Target="slides/slide{i}.xml"/>'
        for i in range(1, slide_count + 1)
    ]
    rels.append(f'<Relationship Id="rId{slide_count+1}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideMaster" Target="slideMasters/slideMaster1.xml"/>')
    z.writestr("ppt/_rels/presentation.xml.rels", f'<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">{"".join(rels)}</Relationships>')

    z.writestr("ppt/slideMasters/slideMaster1.xml", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sldMaster xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
<p:cSld><p:spTree><p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr><p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr></p:spTree></p:cSld>
<p:clrMap bg1="lt1" tx1="dk1" bg2="lt2" tx2="dk2" accent1="accent1" accent2="accent2" accent3="accent3" accent4="accent4" accent5="accent5" accent6="accent6" hlink="hlink" folHlink="folHlink"/>
<p:sldLayoutIdLst><p:sldLayoutId id="2147483649" r:id="rId1"/></p:sldLayoutIdLst>
<p:txStyles><p:titleStyle/><p:bodyStyle/><p:otherStyle/></p:txStyles></p:sldMaster>''')
    z.writestr("ppt/slideMasters/_rels/slideMaster1.xml.rels", '''<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideLayout" Target="../slideLayouts/slideLayout1.xml"/><Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/theme" Target="../theme/theme1.xml"/></Relationships>''')
    z.writestr("ppt/slideLayouts/slideLayout1.xml", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?><p:sldLayout xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main" type="blank" preserve="1"><p:cSld name="Blank"><p:spTree><p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr><p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr></p:spTree></p:cSld><p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr></p:sldLayout>''')
    z.writestr("ppt/slideLayouts/_rels/slideLayout1.xml.rels", '''<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideMaster" Target="../slideMasters/slideMaster1.xml"/></Relationships>''')
    z.writestr("ppt/theme/theme1.xml", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?><a:theme xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" name="ImitationTheme"><a:themeElements><a:clrScheme name="Custom"><a:dk1><a:srgbClr val="1F2937"/></a:dk1><a:lt1><a:srgbClr val="FFFFFF"/></a:lt1><a:dk2><a:srgbClr val="334155"/></a:dk2><a:lt2><a:srgbClr val="F7F9FC"/></a:lt2><a:accent1><a:srgbClr val="2563EB"/></a:accent1><a:accent2><a:srgbClr val="0891B2"/></a:accent2><a:accent3><a:srgbClr val="16A34A"/></a:accent3><a:accent4><a:srgbClr val="EA580C"/></a:accent4><a:accent5><a:srgbClr val="7C3AED"/></a:accent5><a:accent6><a:srgbClr val="DC2626"/></a:accent6><a:hlink><a:srgbClr val="2563EB"/></a:hlink><a:folHlink><a:srgbClr val="7C3AED"/></a:folHlink></a:clrScheme><a:fontScheme name="Custom"><a:majorFont><a:latin typeface="Microsoft YaHei"/><a:ea typeface="Microsoft YaHei"/></a:majorFont><a:minorFont><a:latin typeface="Microsoft YaHei"/><a:ea typeface="Microsoft YaHei"/></a:minorFont></a:fontScheme><a:fmtScheme name="Custom"><a:fillStyleLst><a:solidFill><a:schemeClr val="phClr"/></a:solidFill></a:fillStyleLst><a:lnStyleLst><a:ln w="6350"><a:solidFill><a:schemeClr val="phClr"/></a:solidFill></a:ln></a:lnStyleLst><a:effectStyleLst><a:effectStyle><a:effectLst/></a:effectStyle></a:effectStyleLst><a:bgFillStyleLst><a:solidFill><a:schemeClr val="phClr"/></a:solidFill></a:bgFillStyleLst></a:fmtScheme></a:themeElements></a:theme>''')


def main() -> None:
    slides = build_slides()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(OUT, "w", compression=zipfile.ZIP_DEFLATED) as z:
        write_static_parts(z, len(slides))
        for i, xml in enumerate(slides, start=1):
            z.writestr(f"ppt/slides/slide{i}.xml", xml)
            z.writestr(f"ppt/slides/_rels/slide{i}.xml.rels", '<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideLayout" Target="../slideLayouts/slideLayout1.xml"/></Relationships>')
    print(OUT)


if __name__ == "__main__":
    main()
