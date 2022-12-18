class robot:
	def __init__(self):
		""" 
		physical parameters of the stretch:
			defaultArmDepth: in stowed position, extensipn of gripper tip from base location 
			defaultArmHeight: set to table height
			totalArmDepth: total extension of gripper tip from stowed position 
			totalArmHeight: max height attainable by gripper wrt base
		"""
		self.totalArmDepth = 0.5
		self.totalArmHeight = 1.098
		self.defaultArmDepth = 0.5
		self.defaultArmHeight = 0


