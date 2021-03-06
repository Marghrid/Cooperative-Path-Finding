class Instance:
    def __init__(self, itype, n_agents, o_prob, seed, filename):
        self.type = itype
        self.n_agents = n_agents
        self.o_prob = o_prob
        self.seed = seed
        self.filename = filename.replace(" ", "")

    # print("new Instance: " + self.type + ", " + str(self.n_agents) + \
    #	", " + str(self.o_prob) + ", " + str(self.seed))

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            # return self.type == other.type and \
            #	self.o_prob == other.o_prob and \
            #	self.seed == other.seed and \
            #	self.n_agents == other.n_agents \
            return self.filename == other.filename

        return False

    def __ne__(self, other):
        if isinstance(self, other.__class__):
            return self.type != other.type or \
                   self.o_prob != other.o_prob or \
                   self.seed != other.seed or \
                   self.n_agents != other.n_agents or \
                   self.filename != other.filename

        return True

    def __lt__(self, other):
        return self.filename < other.filename
