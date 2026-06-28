import heapq

class RobotGatePlanner:
    """Path through 3 gates (in order) minimizing 90-degree turns first, then steps.

    Cell (li,num) centred at (li+0.5,num+0.5); Z A B C D E -> li 0..5, num 0..5.
    A gate's gap is the shared edge of two adjacent cells; crossing it (either
    direction) passes the gate. Legs are non-blocking. A 'turn' is a 90-deg heading
    change (1 unit); a 180-deg reversal is 2 units. Cost = turn_cost*turns + step_cost*steps.
    """

    LETTERS = "ZABCDE"
    SIZE = 6
    DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1))   # E, W, N, S in (dli, dnum)

    def __init__(self, g_hint1=None, g_hint2=None, turn_cost=1000, step_cost=1):
        g_hint1 = g_hint1 if g_hint1 is not None else globals().get("g_hint1")
        g_hint2 = g_hint2 if g_hint2 is not None else globals().get("g_hint2")
        self.gates = self._parse(g_hint1, g_hint2)
        self.turn_cost, self.step_cost = turn_cost, step_cost

    @staticmethod
    def _leg(tok):  return int(tok[0]), int(tok[1])
    @staticmethod
    def _edge(a, b): return frozenset((a, b))

    def _parse(self, g1, g2):
        gates = []
        part2, part3 = g2.split("/")
        self.gate_notation = [g1.strip(), part2.strip(), part3.strip()]   # original notation
        x1, y1 = self._leg(g1.split(",")[0])                 # gate 1: horizontal legs
        gates.append(self._edge((x1, y1 - 1), (x1, y1)))
        part2, part3 = g2.split("/")
        a2, b2 = part2.split(",")                            # gate 2: vertical legs
        (x3, ya), (_, yb) = self._leg(a2), self._leg(b2)
        yl = min(ya, yb)
        gates.append(self._edge((x3 - 1, yl), (x3, yl)))
        x5, y5 = self._leg(part3.split(",")[0])              # gate 3: horizontal legs
        gates.append(self._edge((x5, y5 - 1), (x5, y5)))
        return gates

    def label(self, cell):
        return f"{self.LETTERS[cell[0]]}{cell[1]}"

    def gate_labels(self):
        return [" / ".join(sorted(self.label(c) for c in g)) for g in self.gates]

    @staticmethod
    def _turn_units(h, hn):
        if h is None:                                        # first move: free orientation
            return 0
        return {1: 0, 0: 1, -1: 2}[h[0]*hn[0] + h[1]*hn[1]]  # same / perp / reverse

    def solve(self, start=(0, 0)):
        s0 = (start, 0, None)                                # (cell, gates_passed, heading)
        best = {s0: 0}; parent = {s0: None}
        turns_at = {s0: 0}; steps_at = {s0: 0}
        pq = [(0, 0, 0, s0)]
        goal = None
        while pq:
            cost, turns, steps, state = heapq.heappop(pq)
            if cost > best.get(state, float("inf")):
                continue
            cell, passed, h = state
            li, num = cell
            if passed == 3 and (li == 0 or num == 5):        # reached a border end
                goal = state; break
            for hn in self.DIRS:
                nli, nnum = li + hn[0], num + hn[1]
                if not (0 <= nli < self.SIZE and 0 <= nnum < self.SIZE):
                    continue
                ncell = (nli, nnum)
                npassed = passed + 1 if (passed < 3 and
                          self._edge(cell, ncell) == self.gates[passed]) else passed
                tu = self._turn_units(h, hn)
                ncost = cost + self.turn_cost * tu + self.step_cost
                ns = (ncell, npassed, hn)
                if ncost < best.get(ns, float("inf")):
                    best[ns] = ncost; parent[ns] = state
                    turns_at[ns] = turns + tu; steps_at[ns] = steps + 1
                    heapq.heappush(pq, (ncost, turns_at[ns], steps_at[ns], ns))

        if goal is None:
            raise RuntimeError("No solution found.")

        chain = []
        st = goal
        while st is not None:
            chain.append(st); st = parent[st]
        chain.reverse()

        path = [self.label(c) for (c, _, _) in chain]
        gate_step = {i: chain[i][1] for i in range(1, len(chain))
                     if chain[i][1] != chain[i - 1][1]}
        turn_step = {}
        for i in range(2, len(chain)):
            hp, hc = chain[i - 1][2], chain[i][2]
            if hp is not None and hp != hc:
                turn_step[i - 1] = self._turn_units(hp, hc)
        return {"turns": turns_at[goal], "steps": steps_at[goal], "end": self.label(goal[0]),
                "path": path, "gate_pass_step": gate_step, "turn_at_step": turn_step,
                "gates": list(self.gate_notation),}

    def describe(self):
        r = self.solve(); parts = []
        for i, lbl in enumerate(r["path"]):
            tag = (f"[G{r['gate_pass_step'][i]}]" if i in r["gate_pass_step"] else "")
            tag += "*" * r["turn_at_step"].get(i, 0)
            parts.append(lbl + tag)
        return (f"Gates: {r['gates']}\n"
                f"Turns: {r['turns']}  Steps: {r['steps']}  (end {r['end']})\n"
                f"{' -> '.join(parts)}\n"
                f"(* = 90-deg spin, ** = 180-deg reversal, [Gn] = gate n passed)")


def main():
    # Sample input (global string variables)
    global g_hint1, g_hint2
    g_hint1 = "25,35"
    g_hint2 = "53,54/12,22"

    planner = RobotGatePlanner(g_hint1, g_hint2)
    print(planner.describe())


if __name__ == "__main__":
    main()