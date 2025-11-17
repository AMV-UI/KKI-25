from core.utils.config import Direction, Param

class FindMode:
    def __init__(self, node):
        self.node = node
        self.initial_heading = -1
        self.current_heading = -1

        self.threshold = 90

        self.range_low = -1
        self.range_high = -1

        self.track = Param.TRACK.getValue(self.node)
        self.find_status = "Left" if self.track == "A" else "Right"

        self.GO_LEFT = -200  # STATE
        self.GO_RIGHT = 200
        self.dir_map = {
            "Right": self.GO_RIGHT,
            "Left": self.GO_LEFT,
        }

    def set_initial_heading(self, heading):
        """Set the initial heading as reference point"""
        self.initial_heading = heading

    def set_range(self, direction):
        """Set the search range based on direction and track"""
        if self.track == "A":
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

        return self.dir_map[self.find_status]