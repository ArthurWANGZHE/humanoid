from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
import re
import xml.etree.ElementTree as ET


_NAME_PATTERNS = {
    "link": re.compile(r'<link\b[^>]*\bname="([^"]+)"'),
    "joint": re.compile(r'<joint\b[^>]*\bname="([^"]+)"'),
}


@dataclass(frozen=True)
class URDFBlock:
    kind: str
    name: str
    start_line: int
    end_line: int
    lines: tuple[str, ...]


@dataclass(frozen=True)
class URDFInspection:
    path: Path
    exists: bool
    blocks_by_kind: dict[str, dict[str, list[URDFBlock]]]

    @property
    def total_links(self) -> int:
        return sum(len(blocks) for blocks in self.blocks_by_kind["link"].values())

    @property
    def total_joints(self) -> int:
        return sum(len(blocks) for blocks in self.blocks_by_kind["joint"].values())

    @property
    def duplicate_links(self) -> dict[str, list[URDFBlock]]:
        return {
            name: blocks
            for name, blocks in self.blocks_by_kind["link"].items()
            if len(blocks) > 1
        }

    @property
    def duplicate_joints(self) -> dict[str, list[URDFBlock]]:
        return {
            name: blocks
            for name, blocks in self.blocks_by_kind["joint"].items()
            if len(blocks) > 1
        }

    @property
    def link_names(self) -> set[str]:
        return set(self.blocks_by_kind["link"].keys())

    @property
    def joint_names(self) -> set[str]:
        return set(self.blocks_by_kind["joint"].keys())

    @property
    def has_duplicates(self) -> bool:
        return bool(self.duplicate_links or self.duplicate_joints)

    def related_joint_blocks_for_link(self, link_name: str) -> list[URDFBlock]:
        related: list[URDFBlock] = []
        patterns = (
            f'link="{link_name}"',
            f'<parent',
            f'<child',
        )
        for blocks in self.blocks_by_kind["joint"].values():
            for block in blocks:
                block_text = "\n".join(block.lines)
                if patterns[0] not in block_text:
                    continue
                if patterns[1] in block_text or patterns[2] in block_text:
                    related.append(block)
        return related


def _scan_named_blocks(lines: list[str], kind: str) -> dict[str, list[URDFBlock]]:
    tag = kind
    open_tag_pattern = re.compile(rf"<{tag}\b")
    close_tag_pattern = re.compile(rf"</{tag}>")
    name_pattern = _NAME_PATTERNS[kind]
    blocks: dict[str, list[URDFBlock]] = defaultdict(list)

    in_block = False
    block_start = 0
    block_lines: list[str] = []

    for index, line in enumerate(lines, start=1):
        if not in_block and open_tag_pattern.search(line):
            in_block = True
            block_start = index
            block_lines = [line]
            if close_tag_pattern.search(line):
                block_text = "\n".join(block_lines)
                match = name_pattern.search(block_text)
                if match is not None:
                    blocks[match.group(1)].append(
                        URDFBlock(kind=kind, name=match.group(1), start_line=block_start, end_line=index, lines=tuple(block_lines))
                    )
                in_block = False
                block_lines = []
            continue

        if not in_block:
            continue

        block_lines.append(line)
        if not close_tag_pattern.search(line):
            continue

        block_text = "\n".join(block_lines)
        match = name_pattern.search(block_text)
        if match is not None:
            blocks[match.group(1)].append(
                URDFBlock(kind=kind, name=match.group(1), start_line=block_start, end_line=index, lines=tuple(block_lines))
            )
        in_block = False
        block_lines = []

    return dict(blocks)


def inspect_urdf(path: Path) -> URDFInspection:
    resolved_path = Path(path)
    if not resolved_path.exists() or not resolved_path.is_file():
        return URDFInspection(
            path=resolved_path,
            exists=False,
            blocks_by_kind={"link": {}, "joint": {}},
        )

    text_lines = resolved_path.read_text(encoding="utf-8").splitlines()
    return URDFInspection(
        path=resolved_path,
        exists=True,
        blocks_by_kind={
            "link": _scan_named_blocks(text_lines, "link"),
            "joint": _scan_named_blocks(text_lines, "joint"),
        },
    )


def format_block(block: URDFBlock) -> list[str]:
    header = f"  lines {block.start_line}-{block.end_line}:"
    body = [f"    {block.start_line + offset}: {line}" for offset, line in enumerate(block.lines)]
    return [header, *body]


def format_duplicate_report(inspection: URDFInspection) -> str:
    if not inspection.exists:
        return f"[urdf-check] file not found: {inspection.path}"

    lines = [
        f"[urdf-check] path: {inspection.path}",
        f"[urdf-check] total_links: {inspection.total_links}",
        f"[urdf-check] total_joints: {inspection.total_joints}",
    ]

    if not inspection.duplicate_links:
        lines.append("[urdf-check] duplicate links: none")
    else:
        lines.append("[urdf-check] duplicate links:")
        for name in sorted(inspection.duplicate_links):
            blocks = inspection.duplicate_links[name]
            line_list = ", ".join(str(block.start_line) for block in blocks)
            lines.append(f"  - {name}: lines {line_list}")
            for occurrence_index, block in enumerate(blocks, start=1):
                lines.append(f"    occurrence {occurrence_index}")
                lines.extend(format_block(block))
            related_blocks = inspection.related_joint_blocks_for_link(name)
            if related_blocks:
                lines.append("    related joint blocks")
                for block in related_blocks:
                    lines.extend(format_block(block))

    if not inspection.duplicate_joints:
        lines.append("[urdf-check] duplicate joints: none")
    else:
        lines.append("[urdf-check] duplicate joints:")
        for name in sorted(inspection.duplicate_joints):
            blocks = inspection.duplicate_joints[name]
            line_list = ", ".join(str(block.start_line) for block in blocks)
            lines.append(f"  - {name}: lines {line_list}")
            for occurrence_index, block in enumerate(blocks, start=1):
                lines.append(f"    occurrence {occurrence_index}")
                lines.extend(format_block(block))

    return "\n".join(lines)


def validate_urdf_inputs(
    urdf_path: Path,
    root_link: str | None = None,
    end_effector_link: str | None = None,
) -> URDFInspection:
    inspection = inspect_urdf(urdf_path)
    errors: list[str] = []

    if not inspection.exists:
        errors.append(f"URDF file does not exist: {urdf_path}")
    else:
        if inspection.duplicate_links:
            errors.append(
                "Duplicate link names found: "
                + ", ".join(sorted(inspection.duplicate_links))
            )
        if inspection.duplicate_joints:
            errors.append(
                "Duplicate joint names found: "
                + ", ".join(sorted(inspection.duplicate_joints))
            )
        if root_link is not None and root_link not in inspection.link_names:
            errors.append(f"Root link '{root_link}' was not found in {urdf_path}")
        if end_effector_link is not None and end_effector_link not in inspection.link_names:
            errors.append(f"End-effector link '{end_effector_link}' was not found in {urdf_path}")

    if errors:
        raise ValueError("[urdf-check] Lula preflight failed:\n- " + "\n- ".join(errors))

    return inspection


def build_link_chain(urdf_path: Path, root_link: str, end_effector_link: str) -> list[str]:
    root = ET.parse(urdf_path).getroot()
    child_to_parent: dict[str, str] = {}

    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        parent_name = parent.attrib.get("link")
        child_name = child.attrib.get("link")
        if not parent_name or not child_name:
            continue
        existing_parent = child_to_parent.get(child_name)
        if existing_parent is not None and existing_parent != parent_name:
            raise ValueError(
                f"Link '{child_name}' has multiple parents in URDF {urdf_path}: "
                f"{existing_parent}, {parent_name}"
            )
        child_to_parent[child_name] = parent_name

    chain = [end_effector_link]
    current = end_effector_link
    while current != root_link:
        parent_name = child_to_parent.get(current)
        if parent_name is None:
            raise ValueError(
                f"Could not build link chain from '{root_link}' to '{end_effector_link}' in {urdf_path}"
            )
        chain.append(parent_name)
        current = parent_name

    chain.reverse()
    return chain
