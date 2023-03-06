import opensim as osim
from mocolib import *

modelFile = "./MoBL_ARMS_bimanual_6_2_21.osim"


# model = osim.Model(modelFile)
# model.setUseVisualizer(True)

model = getMuscleDrivenModel(modelFile)


study = osim.MocoStudy()

problem = study.updProblem()
problem.setModel(model)
