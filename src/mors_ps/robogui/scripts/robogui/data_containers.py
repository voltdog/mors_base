
class StateData():
    def __init__(self) -> None:
        self.data = []
        # self.data.append(["Desired Values", "Mode", 0, 0])
        # self.data.append(["Desired Values", "Start", 1, 0])

        # self.data.append(["Desired Values", "Robot X vel", 2, 0])
        # self.data.append(["Desired Values", "Robot Y vel", 3, 0])
        # self.data.append(["Desired Values", "Robot Z turn", 4, 0])
        # self.data.append(["Desired Values", "Robot Stride Freq", 5, 0])
        # self.data.append(["Desired Values", "Robot Stride Height", 6, 0])
        # self.data.append(["Desired Values", "Robot Stride Length", 7, 0])
        # self.data.append(["Desired Values", "Robot Gait Type", 8, 0])

        # self.data.append(["Desired Values", "Foot R1 X Pos", 1, 0])
        # self.data.append(["Desired Values", "Start", 1, 0])
        # self.data.append(["Desired Values", "Start", 1, 0])
        # self.data.append(["Desired Values", "Start", 1, 0])
        # self.data.append(["Desired Values", "Start", 1, 0])
        # self.data.append(["Desired Values", "Start", 1, 0])
        # self.data.append(["Desired Values", "Start", 1, 0])
        # self.data.append(["Desired Values", "Start", 1, 0])

        ref_signals = ["Mode", "Start", "Ref Robot X vel", "Ref Robot Y vel", "Ref Robot Z turn", "Ref Robot Stride Freq", "Ref Robot Stride Length", 
                        "Ref Foot R1 X Pos", "Ref Foot L1 X Pos", "Ref Foot R2 X Pos", "Ref Foot L2 X Pos", 
                        "Ref Foot R1 Y Pos", "Ref Foot L1 Y Pos", "Ref Foot R2 Y Pos", "Ref Foot L2 Y Pos", 
                        "Ref Foot R1 Z Pos", "Ref Foot L1 Z Pos", "Ref Foot R2 Z Pos", "Ref Foot L2 Z Pos", 
                        "Ref Body X Pos",    "Ref Body Y Pos",    "Ref Body Z Pos",    "Ref Body X Angle", "Ref Body Y Angle", "Ref Body Z Angle", 
                        "Ref Joint Pos", "Ref Joint Vel", "Ref Joint Torq", "Ref Joint Kp", "Ref Joint Kd",
                        "Cute Action"]
        
        joint_signals = ["Joint Pos", "Joint Vel", "Joint Torq"]
        body_signals  = ["Body Euler X", "Body Euler Y", "Body Euler Z", "Body Acc X", "Body Acc Y", "Body Acc Z"]

        cmd_signals = ["Ref Joint Pos", "Ref Joint Vel", "Ref Joint Torq", "Ref Joint Kp", "Ref Joint Kd"]

        foot_signals = ["Foot R1 X Pos", "Foot R1 Y Pos", "Foot R1 Z Pos", 
                        "Foot L1 X Pos", "Foot L1 Y Pos", "Foot L1 Z Pos", 
                        "Foot R2 X Pos", "Foot R2 Y Pos", "Foot R2 Z Pos", 
                        "Foot L2 X Pos", "Foot L2 Y Pos", "Foot L2 Z Pos"]
        
        for i in range(25):
            self.data.append([0, "Desired Values", ref_signals[i], i, 0])
        for i in range(12):
            self.data.append([0, "Desired Values", f"Ref Joint Pos {i+1}", i+25, 0])
        for i in range(12):
            self.data.append([0, "Desired Values", f"Ref Joint Vel {i+1}", i+37, 0])
        for i in range(12):
            self.data.append([0, "Desired Values", f"Ref Joint Torq {i+1}", i+49, 0])
        for i in range(12):
            self.data.append([0, "Desired Values", f"Ref Joint Kp {i+1}", i+61, 0])
        for i in range(12):
            self.data.append([0, "Desired Values", f"Ref Joint Kd {i+1}", i+73, 0])
        self.data.append([0, "Desired Values", "Cute Action", 85, 0])

        for i in range(len(joint_signals)):
            for j in range(12):
                self.data.append([1, "Joint States", f"{joint_signals[i]} {j+1}", j+i*12+86, 0])

        for i in range(len(body_signals)):
            self.data.append([2, "Body States", body_signals[i], i+122, 0])
        
        for i in range(len(cmd_signals)):
            for j in range(12):
                self.data.append([3, "Commands to Hardware", f"{cmd_signals[i]} {j+1}", j+i*12+128, 0])
        
        for i in range(len(foot_signals)):
            self.data.append([4, "Foot Positions", foot_signals[i], i+188, 0])

class StateDataDict():
    def __init__(self):
        ref_signals = ["Ref Robot X vel", "Ref Robot Y vel", "Ref Robot Z turn",  
                        "Ref Foot R1 X Pos", "Ref Foot L1 X Pos", "Ref Foot R2 X Pos", "Ref Foot L2 X Pos", 
                        "Ref Foot R1 Y Pos", "Ref Foot L1 Y Pos", "Ref Foot R2 Y Pos", "Ref Foot L2 Y Pos", 
                        "Ref Foot R1 Z Pos", "Ref Foot L1 Z Pos", "Ref Foot R2 Z Pos", "Ref Foot L2 Z Pos", 
                        "Ref Body X Pos",    "Ref Body Y Pos",    "Ref Body Z Pos",    "Ref Body X Angle", "Ref Body Y Angle", "Ref Body Z Angle", 
                        ]
        
        joint_signals = ["Joint Pos", "Joint Vel", "Joint Torq"]
        body_signals  = ["Body Euler X", "Body Euler Y", "Body Euler Z", 
                         "Body Acc X", "Body Acc Y", "Body Acc Z", 
                         "Body Quat X", "Body Quat Y", "Body Quat Z", "Body Quat W", 
                         "Body Ang Vel X", "Body Ang Vel Y", "Body Ang Vel Z",
                         "Temperature"]
        cmd_signals = ["Ref Joint Pos", "Ref Joint Vel", "Ref Joint Torq"]
        foot_signals = ["Foot R1 X Pos", "Foot R1 Y Pos", "Foot R1 Z Pos", 
                        "Foot L1 X Pos", "Foot L1 Y Pos", "Foot L1 Z Pos", 
                        "Foot R2 X Pos", "Foot R2 Y Pos", "Foot R2 Z Pos", 
                        "Foot L2 X Pos", "Foot L2 Y Pos", "Foot L2 Z Pos"]

        des_dict1 = {f"{i}": 0 for i in ref_signals}
        des_dict2 = {f"{j} {i}": 0 for j in cmd_signals for i in range(1, 13) }

        self.data = {"Desired Values":
                        {**des_dict1, **des_dict2},
                    "Joint States": 
                        {f"{j} {i}": 0 for j in joint_signals for i in range(1, 13) },
                    "Body States":
                        {f"{i}": 0 for i in body_signals},
                    "Commands to HL from LL":
                        {f"LL {j} {i}": 0 for j in cmd_signals for i in range(1, 13) },
                    "Foot Positions":
                        {f"{i}": 0 for i in foot_signals},
                    }

if __name__ == '__main__':
    state = StateData()
    for i in range(len(state.data)):
        print(state.data[i])