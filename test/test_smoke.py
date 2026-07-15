#!/usr/bin/env python3

import ast
import importlib
import importlib.util
import os
import pathlib
import shutil
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).resolve().parents[1]

EXPECTED_PACKAGES = {
    "baxter_common",
    "baxter_core_msgs",
    "baxter_description",
    "baxter_examples",
    "baxter_gazebo",
    "baxter_interface",
    "baxter_maintenance_msgs",
    "baxter_moveit_config",
    "baxter_sdk",
    "baxter_sim_controllers",
    "baxter_sim_examples",
    "baxter_sim_hardware",
    "baxter_sim_io",
    "baxter_sim_kinematics",
    "baxter_simulator",
    "baxter_tools",
    "rethink_ee_description",
}

IMPORT_MODULES = [
    "baxter_dataflow",
    "baxter_control.pid",
    "baxter_interface",
    "baxter_tools",
    "baxter_examples",
    "baxter_external_devices.getch",
    "joint_trajectory_action.bezier",
    "joint_trajectory_action.minjerk",
    "head_action.head_action",
    "gripper_action.gripper_action",
]

XACRO_FILES = [
    "baxter_common/baxter_description/urdf/baxter.urdf.xacro",
    "baxter_common/rethink_ee_description/urdf/electric_gripper/example_end_effector.urdf.xacro",
    "baxter_common/rethink_ee_description/urdf/pneumatic_gripper/example_end_effector.urdf.xacro",
    "baxter_common/rethink_ee_description/urdf/null_gripper/example_end_effector.urdf.xacro",
    "baxter_moveit_config/config/baxter.srdf.xacro",
]

GENERATED_DYNAMIC_RECONFIGURE_MODULES = {
    "baxter_examples.cfg",
    "baxter_interface.cfg",
}


def _paths(pattern):
    return sorted(p for p in ROOT.rglob(pattern) if ".git" not in p.parts and "__pycache__" not in p.parts)


def _rel(path):
    return str(path.relative_to(ROOT))


def _failures(items, limit=40):
    if len(items) <= limit:
        return "\n".join(items)
    return "\n".join(items[:limit] + ["... %d more" % (len(items) - limit)])


def _add_source_paths():
    for path in sorted(p for p in ROOT.rglob("src") if p.is_dir()):
        sys.path.insert(0, str(path))


def _ros_package_path():
    paths = {str(p.parent) for p in _paths("package.xml")}
    paths.update(str(p) for p in [ROOT, ROOT / "baxter", ROOT / "baxter_common", ROOT / "baxter_simulator"])
    old = os.environ.get("ROS_PACKAGE_PATH")
    if old:
        paths.update(old.split(os.pathsep))
    return os.pathsep.join(sorted(paths))


def _import_errors(path):
    tree = ast.parse(path.read_text(), filename=str(path))
    errors = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names = [(alias.name, None) for alias in node.names]
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            names = [(node.module, [alias.name for alias in node.names])]
        else:
            continue

        for module, imported_names in names:
            if module in GENERATED_DYNAMIC_RECONFIGURE_MODULES:
                continue
            try:
                spec = importlib.util.find_spec(module)
            except (ImportError, AttributeError, ValueError) as exc:
                errors.append("%s:%d: %s (%s)" % (_rel(path), node.lineno, module, exc))
                continue
            if spec is None:
                errors.append("%s:%d: %s" % (_rel(path), node.lineno, module))
                continue
            if module == "operator" and imported_names:
                operator = importlib.import_module("operator")
                for name in imported_names:
                    if name != "*" and not hasattr(operator, name):
                        errors.append("%s:%d: operator.%s" % (_rel(path), node.lineno, name))
    return errors


class SmokeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _add_source_paths()

    def test_package_manifests(self):
        found = set()
        for manifest in _paths("package.xml"):
            root = ET.parse(str(manifest)).getroot()
            name = root.findtext("name")
            self.assertTrue(name, _rel(manifest))
            found.add(name)
        self.assertEqual(EXPECTED_PACKAGES, found)

    def test_python_syntax(self):
        errors = []
        for path in _paths("*.py"):
            try:
                compile(path.read_text(), str(path), "exec")
            except SyntaxError as exc:
                errors.append("%s:%s: %s" % (_rel(path), exc.lineno, exc.msg))
        self.assertFalse(errors, _failures(errors))

    def test_python_imports_are_resolvable(self):
        errors = []
        for path in _paths("*.py"):
            errors.extend(_import_errors(path))
        self.assertFalse(errors, _failures(errors))

    def test_importable_modules(self):
        errors = []
        for module in IMPORT_MODULES:
            try:
                importlib.import_module(module)
            except Exception as exc:
                errors.append("%s: %s" % (module, exc))
        self.assertFalse(errors, _failures(errors))

    def test_launch_files_are_xml(self):
        errors = []
        for launch in _paths("*.launch"):
            try:
                ET.parse(str(launch))
            except ET.ParseError as exc:
                errors.append("%s: %s" % (_rel(launch), exc))
        self.assertFalse(errors, _failures(errors))

    def test_selected_xacro_files_expand(self):
        xacro = shutil.which("xacro")
        self.assertTrue(xacro, "xacro executable not found")

        env = os.environ.copy()
        env["ROS_PACKAGE_PATH"] = _ros_package_path()
        errors = []
        for relpath in XACRO_FILES:
            path = ROOT / relpath
            proc = subprocess.run(
                [xacro, str(path)],
                cwd=str(ROOT),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=20,
            )
            if proc.returncode:
                errors.append("%s: %s" % (relpath, proc.stderr.strip() or proc.stdout.strip()))
                continue
            try:
                ET.fromstring(proc.stdout)
            except ET.ParseError as exc:
                errors.append("%s: %s" % (relpath, exc))
        self.assertFalse(errors, _failures(errors))


if __name__ == "__main__":
    unittest.main()
