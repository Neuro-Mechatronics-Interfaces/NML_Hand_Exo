import time


# training_task.py
class TrainingTask:
    def __init__(self, name, mvc_required, effort_levels, hold_time_sec, rest_time_sec, repetitions):
        self.name = name
        self.mvc_required = mvc_required
        self.effort_levels = effort_levels
        self.hold_time_sec = hold_time_sec
        self.rest_time_sec = rest_time_sec
        self.repetitions = repetitions
        self.results = []

    def run(self, emg_interface, exo_interface):
        if self.mvc_required:
            print(f"[{self.name}] Acquiring MVC baseline...")
            emg_interface.acquire_mvc(self.name)

        for effort in self.effort_levels:
            for rep in range(self.repetitions):
                print(f"[{self.name}] Trial {rep+1}: Target effort = {effort}%")
                emg_interface.display_effort_target(effort)
                exo_interface.assist_movement(self.name, effort)
                self.results.append({
                    'effort': effort,
                    'rep': rep + 1,
                    'timestamp': time.time()
                })
                time.sleep(self.hold_time_sec)
                print("Resting...")
                time.sleep(self.rest_time_sec)

    def export_results(self):
        return self.results
