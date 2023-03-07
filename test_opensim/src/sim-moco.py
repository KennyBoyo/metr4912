import opensim as osim
from mocolib import *

modelFile = "../models/os4bimanual/os4bimanual.osim"


# model = osim.Model(modelFile)
# model.setUseVisualizer(True)

model = getMuscleDrivenModel(modelFile)


study = osim.MocoStudy()

problem = study.updProblem()
problem.setModel(model)
