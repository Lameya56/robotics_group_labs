#!/usr/bin/env python3
import os, re, shutil, sys

root = os.path.expanduser('~/ros2_ws/src/aws-robomaker-small-warehouse-world/models')
if not os.path.isdir(root):
    print("Models root not found:", root); sys.exit(1)

model_files = []
for dp, _, files in os.walk(root):
    if 'model.sdf' in files:
        model_files.append(os.path.join(dp, 'model.sdf'))

re_model_tag   = re.compile(r'(<model\b[^>]*>)', re.IGNORECASE)
re_static_tag  = re.compile(r'<\s*static\s*>.*?</\s*static\s*>', re.IGNORECASE|re.DOTALL)
re_link_block  = re.compile(r'(<link\b[^>]*>)(.*?)(</link>)', re.IGNORECASE|re.DOTALL)
re_inertial    = re.compile(r'<\s*inertial\s*>.*?</\s*inertial\s*>', re.IGNORECASE|re.DOTALL)

SAFE_INERTIAL = (
    "  <inertial>\n"
    "    <mass>1.0</mass>\n"
    "    <inertia>\n"
    "      <ixx>1.0</ixx><iyy>1.0</iyy><izz>1.0</izz>\n"
    "      <ixy>0.0</ixy><ixz>0.0</ixz><iyz>0.0</iyz>\n"
    "    </inertia>\n"
    "  </inertial>\n"
)

patched_files = 0
for f in model_files:
    with open(f, 'r', encoding='utf-8') as fh:
        txt = fh.read()
    orig = txt

    m = re_model_tag.search(txt)
    if m:
        head, tail = txt[:m.end()], txt[m.end():]
        tail = re_static_tag.sub('', tail)
        txt = head + "\n  <static>true</static>\n" + tail

    def repl(match):
        open_tag, body, close_tag = match.groups()
        body2 = re_inertial.sub('', body)  # strip any existing inertial
        return open_tag + SAFE_INERTIAL + body2 + close_tag

    txt = re_link_block.sub(repl, txt)

    if txt != orig:
        shutil.copy2(f, f + '.bak')
        with open(f, 'w', encoding='utf-8') as fh:
            fh.write(txt)
        patched_files += 1
        print("Patched:", f)

print(f"Patched {patched_files} model files")
