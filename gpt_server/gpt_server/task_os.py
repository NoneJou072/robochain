import queue


class TaskQueue:
    def __init__(self) -> None:
        max_size = 1000
        self.task_os = queue.Queue(max_size)

    def push(self, task):
        self.task_os.put(task)

    def get(self):
        return self.task_os.get()

    @property
    def qsize(self):
        return self.task_os.qsize()
    
    @property
    def empty(self):
        return self.task_os.empty()
