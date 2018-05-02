import statistics

class Cell:
    def __init__(self, encoding, search, line):
        self.encoding = encoding
        self.search = search
        self.line = line

        self.solutions = []
        #print("new Cell: " + self.search + ", " + self.encoding)

    def belongs(self, solution):
        return solution.encoding == self.encoding and \
            solution.search == self.search

    def add_solution(self, solution):
        if not self.belongs(solution):
            raise ValueError("Solution doesn't belong in cell")
        
        self.solutions += [solution]
        self.solutions.sort()

    
    def get_average_CPUtime(self):
        CPUtimes = []
        for solution in self.solutions:
            CPUtimes += [solution.CPUtime]

        return sum(CPUtimes)/len(CPUtimes)

    def get_median_CPUtime(self):
        CPUtimes = []
        for solution in self.solutions:
            CPUtimes += [solution.CPUtime]
        if len(CPUtimes) > 0:
            return statistics.median(CPUtimes)

        return "-"

    def get_n_solved_instances(self):
        solved = 0
        for solution in self.solutions:
            if solution.status == 1 or solution.status == 2:
                solved += 1
        return solved

    def get_n_SAT_instances(self):
        solved = 0
        for solution in self.solutions:
            if solution.status == 3:
                solved += 1
        return solved

    def get_average_optimal_makespan(self):
        makespans = []
        for solution in self.solutions:
            if solution.makespan > -1:
                makespans += [solution.makespan]
                
        if len(makespans) > 0:
            return statistics.median(makespans)

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            return self.encoding == other.encoding and \
                self.search == other.search
        return False

    def __ne__(self, other):
        if isinstance(self, other.__class__):
            return self.encoding != other.encoding and \
                self.search != other.search
        return True

    def __lt__(self, other):
        if self.encoding == other.encoding:
            return self.search > other.search
                # Because I like seaches ordered like UNSAT-SAT, SAT-UNSAT, BINARY
        else:
            return self.encoding < self.encoding
