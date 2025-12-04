from core.utils.config import Direction, Param

class FindMode:
    def __init__(self, node, arena, name):
        self.node = node
        self.initial_heading = 0
        self.current_heading = -1

        self.threshold = 45

        self.range_low = -1
        self.range_high = -1

        self.arena = arena
        self.find_status = "Left" if self.arena == "A" else "Right"

        self.GO_LEFT = (-1 if self.arena == "B" else 1)  
        self.GO_RIGHT = (-1 if self.arena == "A" else 1)  
        self.dir_map = {
            "Right": self.GO_RIGHT,
            "Left": self.GO_LEFT,
        }

        if("0" in name):
            self.set_range(0)
        elif("1" in name):
            self.set_range(1)
        elif("2" in name):
            self.set_range(2)
        elif("3" in name):
            self.set_range(3)

    def set_initial_heading(self, heading):
        """Set the initial heading as reference point"""
        self.initial_heading = heading

    def set_range(self, direction):
        """Set the search range based on direction and track"""
        if self.arena == "A":
            self.range_low = Direction.A[direction] - self.threshold
            self.range_high = Direction.A[direction] + self.threshold
        else:
            self.range_low = Direction.B[direction] - self.threshold
            self.range_high = Direction.B[direction] + self.threshold

    def get_heading(self, raw_heading):
        """Get adjusted heading based on initial heading"""
        heading = raw_heading - self.initial_heading
        if heading < 0:
            heading = 360 + heading
        return heading

    def get_state(self, raw_heading):
        """Get the direction state (GO_LEFT / GO_RIGHT) based on current heading and range"""
        self.current_heading = self.get_heading(raw_heading)
        lb = abs(self.current_heading - self.range_low)
        if self.range_low == 0:
            lb = min(lb, abs(self.current_heading - 360))

        ub = abs(self.current_heading - self.range_high)
        if self.range_high == 0:
            ub = min(ub, abs(self.current_heading - 360))

        if (
            not (self.range_low if self.range_low == 360 else 0)
            < self.current_heading
            < (self.range_high if self.range_high != 0 else 360)
        ):
            if lb < ub:
                self.find_status = "Right"
            else:
                self.find_status = "Left"

        self.node.get_logger(self.f"range low {self.range_low}, range high {self.range_high}")

        return self.dir_map[self.find_status]