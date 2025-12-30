# DEBUG TOOL: record qpos/qvel over time for selected joints
class JointLogger:
    def __init__(self, physics, joint_names):
        self.physics = physics
        self.joint_ids = [physics.model.joint(j).id for j in joint_names]
        self.joint_names = joint_names
        self.time_log = []
        self.qpos_log = {j: [] for j in joint_names}
        self.qvel_log = {j: [] for j in joint_names}

    def record(self):
        t = self.physics.data.time
        self.time_log.append(t)
        for name, jid in zip(self.joint_names, self.joint_ids):
            self.qpos_log[name].append(float(self.physics.data.qpos[jid]))
            self.qvel_log[name].append(float(self.physics.data.qvel[jid]))

    def plot(self):
        import matplotlib.pyplot as plt
        # qpos
        plt.figure(figsize=(10,5))
        for name in self.joint_names:
            plt.plot(self.time_log, self.qpos_log[name], label=name)
        plt.title("qpos over time")
        plt.legend(); plt.show()
        # qvel
        plt.figure(figsize=(10,5))
        for name in self.joint_names:
            plt.plot(self.time_log, self.qvel_log[name], label=name)
        plt.title("qvel over time")
        plt.legend(); plt.show()
