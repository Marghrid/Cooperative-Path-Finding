from Cell import Cell

class Line:
    def __init__(self, itype, n_agents, table):
        self.type = itype
        self.n_agents = n_agents
        self.table = table

        self.cells = []
        for encoding in table.diff_encodings:
            for search in table.diff_searches:
                cell = Cell(encoding, search, self)
                self.cells += [cell]

    # print("new Line: " + self.type + ", " + str(self.n_agents))

    def belongs(self, solution):
        return solution.instance.type == self.type and \
               solution.instance.n_agents == self.n_agents

    def add_encoding(self, encoding):
        for search in self.table.diff_searches:
            cell = Cell(encoding, search, self)
            self.cells += [cell]

    def add_search(self, search):
        for encoding in self.table.diff_encodings:
            cell = Cell(encoding, search, self)
            self.cells += [cell]

    def add_solution(self, solution):
        # if solution doesn't belong in this line, there's nothing to be done.
        if not self.belongs(solution):
            raise ValueError("Solution doesn't belong in line")

        # if solution belongs in one of the previously created cells, then add it to that cell
        for cell in self.cells:
            if cell.belongs(solution) and solution not in cell.solutions:
                cell.add_solution(solution)
                return

        # there should be no solutions unfit for a line's cells
        print("Existing cells in line " + self.type + ", " + str(self.n_agents))
        for cell in self.cells:
            print("cell: " + cell.encoding + ", " + cell.search)
        raise ValueError("Solution does not fit any cell. Unknown encoding " + \
                         solution.encoding + " or search " + solution.search)

    def __str__(self):
        self.cells.sort()
        sep = 16  # number of spaces from start of one field to start of another

        line = self.type + "," + (sep - len(self.type)) * " " + str(self.n_agents)
        last_written = self.n_agents
        for cell in self.cells:
            # line += "," + (sep-len(str(last_written)))*" " + str(cell.get_n_solved_instances()) + \
            #	" of " + str(self.table.get_n_instances_of_type(self.type, self.n_agents))
            # last_written = str(cell.get_n_solved_instances()) + \
            #	" of " + str(self.table.get_n_instances_of_type(self.type, self.n_agents))
            # line += "," + (sep-len(str(last_written)))*" " + str(cell.get_median_CPUtime())
            # last_written = str(cell.get_median_CPUtime())
            if cell.get_average_optimal_makespan() == None:
                continue

            line += ", " + str(cell.get_n_solved_instances()) + \
                    ", " + str(self.table.get_n_instances_of_type(self.type, self.n_agents)) + \
                    ", " + str(float(cell.get_average_optimal_makespan()))

        return line

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            return self.type == other.type and \
                   self.n_agents == other.n_agents

        return False

    def __ne__(self, other):
        if isinstance(self, other.__class__):
            return self.type != itype and \
                   self.n_agents != n_agents

        return True

    def __lt__(self, other):
        if self.type == other.type:
            return self.n_agents < other.n_agents
        else:
            return self.type < other.type
