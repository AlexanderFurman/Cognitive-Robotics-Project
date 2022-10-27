class JointState:
    def __init__(self, val = 0, prev = None, next = None):
        self.value = val
        self.previus = prev
        self.next = next


    def set_state(self, val):
        self.value = val
