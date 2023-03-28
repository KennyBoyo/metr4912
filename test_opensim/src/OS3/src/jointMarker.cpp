#include <OpenSim/OpenSim.h>

using namespace OpenSim;

int main() {
    // Load the OpenSim model
    Model model("my_model.osim");

    // Get the number of joints in the model
    int num_joints = model.getNumJoints();

    // Loop through each joint in the model
    for (int i = 0; i < num_joints; i++) {
        // Get the joint object
        Joint& joint = model.getJointSet().get(i);

        // Get the parent and child bodies of the joint
        const PhysicalFrame& parent_body = joint.getParentFrame();
        const PhysicalFrame& child_body = joint.getChildFrame();

        // Compute the midpoint between the parent and child bodies
        Vec3 midpoint = (parent_body.getMassCenter() + child_body.getMassCenter()) / 2.0;

        // Create a new marker at the midpoint
        std::string marker_name = joint.getName() + "_marker";
        Marker marker(marker_name, *parent_body.clone(), midpoint);

        // Add the marker to the model's marker set
        model.getMarkerSet().adoptAndAppend(&marker);
    }

    // Save the modified model
    model.print("my_model_with_markers.osim");

    return 0;
}
