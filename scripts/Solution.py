class Solution:
	def __init__(self, instance, encoding, search, CPUtime, status):
		self.instance = instance
		self.encoding = encoding
		self.search   = search
		self.CPUtime  = CPUtime
		self.status   = status

		#print("new Solution: " + self.encoding + ", " + self.search + ", " + str(self.CPUtime))

	def __eq__(self, other):
		if isinstance(self, other.__class__):
			return self.instance == instance and \
				self.encoding == encoding and \
				self.search   == search and \
				self.CPUtime  == CPUtime
		return False

	def __ne__(self, other):
		if isinstance(self, other.__class__):
			return self.instance != instance and \
				self.encoding != encoding and \
				self.search   != search and \
				self.CPUtime  != CPUtime

		return True

	def __lt__(self, other):
		if self.instance == other.instance:
			if self.encoding == other.encoding:
				if self.search == other.search:
					return self.CPUtime < other.CPUtime
				else:
					return self.search > other.search
					# Because I like seaches ordered like UNSAT-SAT, SAT-UNSAT, BINARY
			else:
				return self.encoding < self.encoding

		else:
			return self.instance < other.instance
