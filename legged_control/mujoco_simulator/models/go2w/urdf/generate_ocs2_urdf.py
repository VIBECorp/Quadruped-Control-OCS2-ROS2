#!/usr/bin/env python3
"""Generate the OCS2/pinocchio URDF for Go2W from the Unitree go2w_description.

The OCS2 legged_control fork (this repo) expects, per robot:
  - 12 actuated leg joints named with the *canonical* OCS2 names
        LF_HAA LF_HFE LF_KFE  LH_HAA LH_HFE LH_KFE
        RF_HAA RF_HFE RF_KFE  RH_HAA RH_HFE RH_KFE
    (hard-coded in motion_control/.../common/ModelSettings.h). The mapping to the
    Unitree go2w leg labels is  LF=FL, LH=RL(rear-left), RF=FR, RH=RR(rear-right).
  - 4 contact frames named  LF_FOOT RF_FOOT LH_FOOT RH_FOOT  (contactNames3DoF).
  - A massless "base" root link connected by a floating "floating_base" joint to
    the real body link "trunk" (exactly the a2 model structure). createPinocchioInterface
    adds a free-flyer at the URDF root and FIXES every joint that is not one of the 12
    jointNames, so the 4 wheels, foot-motors and the floating_base joint are all welded
    automatically -> the Go2W reduces to a 12-DoF quadruped whose feet are the wheel
    ground-contact points (use footRadius = wheel radius in task.info).

The joint DECLARATION ORDER below (LF, LH, RF, RH) reproduces the a2 model, so the
MPC's joint_control_data / state vector ordering is  [FL, RL, FR, RR]  (each hip,thigh,calf).

Run:  python3 generate_ocs2_urdf.py
Writes robot.urdf next to this script.
"""
import os
import re

SRC = "/home/gpu/Git_krri/unitree_ros/robots/go2w_description/urdf/go2w_description.urdf"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot.urdf")

# Go2W wheel rolling radius [m] (yaw_gait.WHEEL_RADIUS). The contact frame is placed
# at the wheel CENTER; the rolling contact is footRadius below it (set in task.info).
WHEEL_RADIUS = 0.0889

# canonical-joint -> (source Unitree leg prefix, child link, parent link, hip|thigh|calf)
# source leg prefix: FL(front-left)=LF, RL(rear-left)=LH, FR(front-right)=RF, RR(rear-right)=RH
LEGS = [
    ("LF", "FL"),  # canonical LF  <- source FL
    ("LH", "RL"),  # canonical LH  <- source RL
    ("RF", "FR"),  # canonical RF  <- source FR
    ("RH", "RR"),  # canonical RH  <- source RR
]
SEG = [("HAA", "hip"), ("HFE", "thigh"), ("KFE", "calf")]


def read(path):
    with open(path) as f:
        return f.read()


def link_inertial(src, link_name):
    """Return the verbatim <inertial>...</inertial> block of a link, or None."""
    m = re.search(r'<link\s+name="%s"\s*>(.*?)</link>' % re.escape(link_name), src, re.S)
    if not m:
        return None
    body = m.group(1)
    mi = re.search(r"<inertial>.*?</inertial>", body, re.S)
    return mi.group(0) if mi else None


def joint_block(src, joint_name):
    m = re.search(r'<joint\s+name="%s"\s+type="([^"]+)"\s*>(.*?)</joint>' % re.escape(joint_name), src, re.S)
    if not m:
        return None
    typ, body = m.group(1), m.group(2)
    org = re.search(r"<origin([^/]*?)/>", body, re.S)
    axis = re.search(r'<axis\s+xyz="([^"]+)"', body)
    lim = re.search(r"<limit([^/]*?)/>", body, re.S)
    return {
        "type": typ,
        "origin": (org.group(1).strip() if org else 'xyz="0 0 0" rpy="0 0 0"').replace("\n", " "),
        "axis": axis.group(1) if axis else None,
        "limit": (lim.group(1).strip() if lim else None),
    }


def indent(block, pad):
    return "\n".join((pad + ln.strip()) for ln in block.strip().splitlines())


def main():
    src = read(SRC)
    out = []
    out.append('<?xml version="1.0" ?>')
    out.append('<robot name="go2w">')

    # ---- massless root "base" + floating joint to "trunk" (mirror a2 structure) ----
    out.append('  <link name="base">')
    out.append('    <visual><origin rpy="0 0 0" xyz="0 0 0"/>'
               '<geometry><box size="0.001 0.001 0.001"/></geometry></visual>')
    out.append('  </link>')
    out.append('  <joint name="floating_base" type="floating">')
    out.append('    <origin rpy="0 0 0" xyz="0 0 0"/>')
    out.append('    <parent link="base"/>')
    out.append('    <child link="trunk"/>')
    out.append('  </joint>')

    # ---- trunk = source "base" body link (rename), keep its inertial ----
    trunk_inert = link_inertial(src, "base")
    assert trunk_inert, "source base inertial not found"
    out.append('  <link name="trunk">')
    out.append(indent(trunk_inert, "    "))
    out.append('    <visual><origin rpy="0 0 0" xyz="0 0 0"/>'
               '<geometry><box size="0.3762 0.0935 0.114"/></geometry></visual>')
    out.append('  </link>')

    # ---- legs ----
    for canon, srcp in LEGS:
        # links: hip, thigh, calf (+ foot_motor + foot kept as fixed children for inertia)
        parents = {"hip": "trunk", "thigh": "%s_hip" % srcp, "calf": "%s_thigh" % srcp}
        for code, seg in SEG:
            jn_src = "%s_%s_joint" % (srcp, seg)
            jb = joint_block(src, jn_src)
            assert jb, "missing source joint %s" % jn_src
            link = "%s_%s" % (srcp, seg)          # keep Unitree-style link name
            jname = "%s_%s" % (canon, code)        # canonical joint name
            inert = link_inertial(src, link)
            out.append('  <link name="%s">' % link)
            if inert:
                out.append(indent(inert, "    "))
            out.append('  </link>')
            out.append('  <joint name="%s" type="revolute">' % jname)
            out.append('    <origin %s/>' % jb["origin"])
            out.append('    <parent link="%s"/>' % parents[seg])
            out.append('    <child link="%s"/>' % link)
            out.append('    <axis xyz="%s"/>' % (jb["axis"] or "0 1 0"))
            out.append('    <limit %s/>' % (jb["limit"] or 'effort="40" velocity="30" lower="-3.14" upper="3.14"'))
            out.append('  </joint>')

        # foot_motor (fixed, keep inertia ~0.646 kg)
        fm_inert = link_inertial(src, "%s_foot_motor" % srcp)
        fmj = joint_block(src, "%s_foot_motor_joint" % srcp)
        if fm_inert and fmj:
            out.append('  <link name="%s_foot_motor">' % srcp)
            out.append(indent(fm_inert, "    "))
            out.append('  </link>')
            out.append('  <joint name="%s_foot_motor_joint" type="fixed">' % srcp)
            out.append('    <origin %s/>' % fmj["origin"])
            out.append('    <parent link="%s_calf"/>' % srcp)
            out.append('    <child link="%s_foot_motor"/>' % srcp)
            out.append('  </joint>')

        # wheel "foot" (FIXED here -> static contact; keep inertia ~0.520 kg)
        w_inert = link_inertial(src, "%s_foot" % srcp)
        wj = joint_block(src, "%s_foot_joint" % srcp)
        out.append('  <link name="%s_foot">' % srcp)
        if w_inert:
            out.append(indent(w_inert, "    "))
        out.append('  </link>')
        out.append('  <joint name="%s_foot_joint" type="fixed">' % srcp)
        out.append('    <origin %s/>' % (wj["origin"] if wj else 'xyz="-0.07481 0 -0.27443" rpy="0 0 0"'))
        out.append('    <parent link="%s_calf"/>' % srcp)
        out.append('    <child link="%s_foot"/>' % srcp)
        out.append('  </joint>')

        # contact frame *_FOOT at the wheel CENTER (calf + [0,0,-0.2264]); footRadius in task.info
        out.append('  <link name="%s_FOOT"/>' % canon)
        out.append('  <joint name="%s_foot_fixed" type="fixed" dont_collapse="true">' % canon)
        out.append('    <origin rpy="0 0 0" xyz="-0.07481 0 -0.27443"/>')
        out.append('    <parent link="%s_calf"/>' % srcp)
        out.append('    <child link="%s_FOOT"/>' % canon)
        out.append('  </joint>')

    out.append('</robot>')
    text = "\n".join(out) + "\n"
    with open(OUT, "w") as f:
        f.write(text)
    print("wrote", OUT, "(%d lines)" % text.count("\n"))


if __name__ == "__main__":
    main()
