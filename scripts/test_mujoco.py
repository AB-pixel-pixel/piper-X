import mujoco
import numpy as np


def main() -> None:
    xml = """
<mujoco model="minimal">
  <worldbody>
    <body name="box" pos="0 0 0.2">
      <joint name="free" type="free"/>
      <geom type="box" size="0.05 0.05 0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    for _ in range(100):
        mujoco.mj_step(model, data)
    qpos = np.array(data.qpos)
    print("mujoco_ok", float(data.time), qpos.shape)


if __name__ == "__main__":
    main()
