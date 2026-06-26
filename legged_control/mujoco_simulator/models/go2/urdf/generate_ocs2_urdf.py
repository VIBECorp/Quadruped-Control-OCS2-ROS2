#!/usr/bin/env python3
"""Generate the OCS2/pinocchio URDF for Go2 from the Unitree go2_description.

Same approach as the go2w generator, but Go2 has no wheels: the foot is the calf-end
contact (calf -> *_foot, z=-0.213). We keep ALL source links (incl. the motor rotor links,
~1.07 kg total) so the centroidal mass/inertia is correct; OCS2's createPinocchioInterface
fixes every joint not in the 12 canonical jointNames (rotors, calflower, foot, cosmetics),
merging their inertia into the parent.

Leg joints are renamed to the canonical OCS2 names and DECLARED in the order
[LF, LH, RF, RH] = source [FL, RL, FR, RR] so the MPC state / joint_control_data ordering
matches a2/go2w (and the mpc_lowcmd_bridge permutation). Contact frames LF_FOOT/RF_FOOT/
LH_FOOT/RH_FOOT are added at the foot position; use footRadius ~0.02 in task.info.

Run:  python3 generate_ocs2_urdf.py   (writes robot.urdf next to this script)
"""
import os
import re

SRC = "/home/gpu/Git_krri/unitree_ros/robots/go2_description/urdf/go2_description.urdf"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot.urdf")

# canonical OCS2 leg -> source leg label, declared in OCS2 order [LF, LH, RF, RH]
LEGS = [("LF", "FL"), ("LH", "RL"), ("RF", "FR"), ("RH", "RR")]
SEG = [("hip", "HAA"), ("thigh", "HFE"), ("calf", "KFE")]
FOOT_Z = -0.213  # source *_foot_joint origin (calf -> foot)


def read(path):
    with open(path) as f:
        return f.read()


def parse_blocks(src, tag):
    """Return dict name -> full '<tag ...>...</tag>' block."""
    out = {}
    for m in re.finditer(r'<%s\s+name="([^"]+)"[^>]*>.*?</%s>' % (tag, tag), src, re.S):
        out[m.group(1)] = m.group(0)
    # self-closing / empty links like <link name="x"/>
    for m in re.finditer(r'<%s\s+name="([^"]+)"\s*/>' % tag, src):
        out.setdefault(m.group(1), m.group(0))
    return out


def joint_records(src):
    recs = []
    for m in re.finditer(r'<joint\s+name="([^"]+)"\s+type="([^"]+)"[^>]*>(.*?)</joint>', src, re.S):
        name, typ, body = m.group(1), m.group(2), m.group(3)
        chi = re.search(r'<child\s+link="([^"]+)"', body)
        recs.append({"name": name, "type": typ, "child": chi.group(1) if chi else None,
                     "block": m.group(0)})
    return recs


def main():
    src = read(SRC)
    links = parse_blocks(src, "link")
    joints = joint_records(src)
    main_names = {f"{s}_{seg}_joint" for _, s in LEGS for seg, _ in SEG}
    canon_of = {f"{s}_{seg}_joint": f"{c}_{code}" for c, s in LEGS for seg, code in SEG}

    out = ['<?xml version="1.0" ?>', '<robot name="go2">']

    # massless root + floating joint to the real body link "trunk" (mirrors a2 / go2w)
    out.append('  <link name="base">')
    out.append('    <visual><origin rpy="0 0 0" xyz="0 0 0"/>'
               '<geometry><box size="0.001 0.001 0.001"/></geometry></visual>')
    out.append('  </link>')
    out.append('  <joint name="floating_base" type="floating">')
    out.append('    <origin rpy="0 0 0" xyz="0 0 0"/><parent link="base"/><child link="trunk"/>')
    out.append('  </joint>')

    # trunk = source "base" body link, renamed
    trunk = links["base"].replace('name="base"', 'name="trunk"', 1)
    out.append("  " + trunk.strip())

    def emit_joint(block, name):
        b = block
        if name in canon_of:                       # rename the 12 actuated leg joints
            b = b.replace(f'name="{name}"', f'name="{canon_of[name]}"', 1)
        b = b.replace('<parent link="base"/>', '<parent link="trunk"/>')
        b = b.replace('link="base"', 'link="trunk"')   # any other base parent refs
        out.append("  " + b.strip())

    emitted_links = {"base", "trunk"}
    for canon, srcp in LEGS:
        leg_joints = [j for j in joints if j["name"].startswith(srcp + "_")]
        # hip joint first (so trunk's actuated children stay in LF,LH,RF,RH order), then rest
        leg_joints.sort(key=lambda j: (0 if j["name"] == f"{srcp}_hip_joint" else 1))
        for j in leg_joints:
            emit_joint(j["block"], j["name"])
            ch = j["child"]
            if ch and ch in links and ch not in emitted_links:
                out.append("  " + links[ch].strip())
                emitted_links.add(ch)
        # contact frame at the foot position
        out.append(f'  <link name="{canon}_FOOT"/>')
        out.append(f'  <joint name="{canon}_foot_fixed" type="fixed" dont_collapse="true">')
        out.append(f'    <origin rpy="0 0 0" xyz="0 0 {FOOT_Z}"/>'
                   f'<parent link="{srcp}_calf"/><child link="{canon}_FOOT"/>')
        out.append('  </joint>')

    out.append('</robot>')
    text = "\n".join(out) + "\n"
    with open(OUT, "w") as f:
        f.write(text)
    masses = [float(x) for x in re.findall(r'<mass value="([^"]+)"', text)]
    print("wrote", OUT, "total_mass=", round(sum(masses), 3))


if __name__ == "__main__":
    main()
