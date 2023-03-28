#include <OpenSim/OpenSim.h>


// Callback for subscriber which reads the Panda end effector force output topic
OpenSim::ExternalLoads pandaEELoadCallback(Force f) {
    OpenSim::ExternalLoads externalLoads;

    // Extract point of force
    OpenSim::Vec3 pos(0);
    // Extract direction of force
    OpenSim::Vec3 dir(0);

    OpenSim::ConstantForce stepForce(
        "panda_EE_load",  // Name of the force
        100,               // Magnitude of the force (in Newtons)
        pos,  // Point at which the force is applied (in the body's local coordinates)
        dir // Direction of the force (in the body's local coordinates)
    );

    externalLoads.addForce(&stepForce);
    this.externalLoads = externalLoads;
    return externalLoads;
}

// Define a function to compute the external load at each time step
double kinematicDataCallback(MarkerPosition[] time) {
    // Compute the external load as a function of time
    double kinematics = sin(time); // Replace with your own function
    this.kinematics = kinematics
    return kinematics;
}

void solveStep(OpenSim::InverseDynamicsSolver* idSolver, OpenSim::InverseKinematicsSolver* ikSolver, OpenSim::Vec3 force, OpenSim::Vec3* jointPositions) {

}

int main()
{
    // Load the arm model
    OpenSim::Model model("os4bimanual.osim");

    // Create an inverse dynamics solver
    OpenSim::InverseDynamicsSolver IDSolver(model);

    // Set the ExternalLoads component as a force source for the model
    model.addForce(&this.externalLoads);

    // Set up a SimbodyEngine for real-time simulation
    OpenSim::SimbodyEngine simbodyEngine(model);

    // Solve for the inverse dynamics
    OpenSim::Storage inverseDynamicsResults = solver.solve();

    // Iterate over the time range and compute inverse dynamics at each time step
    while (1) {
        // Set the joint angles and velocities for the current time
        solver.setStartTime(this.prevTime);
        solver.setEndTime(this.currTime);
        solver.setDT(this.currTime - this.prevTime);
        solver.setInputData(this.kinematics);

        // Set the external loads for the current time
        externalLoads.setTime(this.prevTime);

        // Solve for the inverse dynamics at the current time
        OpenSim::Array<double> jointTorques;
        model.getMultibodySystem().realize(model.getWorkingState());
        model.getMultibodySystem().realize(model.getWorkingState(), OpenSim::Stage::Dynamics);
        model.getMultibodySystem().realize(model.getWorkingState(), OpenSim::Stage::Acceleration);
        model.getMultibodySystem().realize(model.getWorkingState(), OpenSim::Stage::Report);
        model.getMatterSubsystem().calcResidualForceIgnoringConstraints(
            model.getWorkingState(), 
            jointTorques);

        // Apply the computed joint torques to the model
        simbodyEngine.setControls(jointTorques);

        // Integrate the model forward in time
        simbodyEngine.integrate(nextTime);
        
        // Save the inverse dynamics results for the current time
        inverseDynamicsResults.append(this.prevTime, jointTorques);
    }

    // Save the results to a file
    inverseDynamicsResults.print("armInverseDynamics.sto");

    return 0;
}
