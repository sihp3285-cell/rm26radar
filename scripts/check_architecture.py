#!/usr/bin/env python3
"""Check package boundaries and local dependency cycles without building ROS/GPU code."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
packages = {p.parent.name: p.parent for p in (ROOT / 'src').glob('*/package.xml')}
# Cross-package implementation imports are limited to contracts. Nodes communicate via ROS.
contracts = {'radar27_interfaces', 'rm_field'}
errors = []
graph = {}
for name, path in packages.items():
    xml = ET.parse(path / 'package.xml').getroot()
    dependencies = {e.text for e in xml if e.tag.endswith('depend')}
    graph[name] = dependencies & packages.keys()
    for source in list(path.rglob('*.hpp')) + list(path.rglob('*.cpp')) + list(path.rglob('*.cu')):
        if 'vendor' in source.parts:
            continue
        text = source.read_text()
        for owner in re.findall(r'#include\s*[<"]([^/">]+)/', text):
            if owner in packages and owner != name:
                if owner not in contracts:
                    errors.append(f'{source.relative_to(ROOT)} imports implementation of {owner}')
                if owner not in dependencies:
                    errors.append(f'{name} does not declare {owner}')
        if name == 'rm_field' and re.search(r'#include\s*[<"](?:opencv|rclcpp|NvInfer)', text):
            errors.append(f'domain contract imports runtime library: {source.name}')
    if name not in {'radar27_detection', 'radar27_localization', 'radar27_visualization'}:
        cmake = (path / 'CMakeLists.txt').read_text()
        if re.search(r'find_package\((?:CUDAToolkit|Open3D|Qt5)|enable_language\(CUDA\)', cmake):
            errors.append(f'{name} unexpectedly requires GPU/UI build dependencies')

def visit(name, stack, done):
    if name in stack:
        errors.append('package dependency cycle: ' + ' -> '.join(stack + [name]))
        return
    if name in done:
        return
    for dep in graph[name]:
        visit(dep, stack + [name], done)
    done.add(name)

done = set()
for name in graph:
    visit(name, [], done)
if errors:
    raise SystemExit('\n'.join(errors))
print(f'PASS: {len(packages)} packages; no implementation imports, dependency cycles or misplaced GPU/UI requirements')
