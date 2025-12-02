from setup.dh_inputs import collect_dh_table
from setup.offset_inputs import collect_offsets
from core.robot_model import RobotModel
from ui.robot_ui import RobotUI

def main():
    dh = collect_dh_table()
    offsets = collect_offsets()
    model = RobotModel(dh, offsets)
    ui = RobotUI(model)
    ui.run()

if __name__ == "__main__":
    main()
