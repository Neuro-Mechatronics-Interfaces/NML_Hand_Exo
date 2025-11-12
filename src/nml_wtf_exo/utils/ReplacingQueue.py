from collections import deque

class ReplacingQueue:
    def __init__(self, maxsize):
        self.maxsize = maxsize
        self.queue = deque(maxlen=maxsize)

    def put(self, item):
        """
        Adds an item to the queue. If the queue is full, the oldest item is removed.
        """
        if len(self.queue) == self.maxsize:
            self.queue.popleft()  # Remove the oldest item
        self.queue.append(item)

    def get(self):
        """
        Removes and returns an item from the front of the queue.
        Raises IndexError if the queue is empty.
        """
        if not self.queue:
            raise IndexError("Queue is empty")
        return self.queue.popleft()

    def qsize(self):
        """
        Returns the current size of the queue.
        """
        return len(self.queue)

    def empty(self):
        """
        Checks if the queue is empty.
        """
        return len(self.queue) == 0

    def full(self):
        """
        Checks if the queue is full.
        """
        return len(self.queue) == self.maxsize