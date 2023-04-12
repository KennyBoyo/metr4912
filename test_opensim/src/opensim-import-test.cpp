#include <OpenSim/OpenSim.h>
using namespace SimTK;
using namespace OpenSim;

int main() {
    // Model model = Model("$HOME/Downloads/MobL_ARMS_OpenSim41_unimanual_tutorial/MoBL-ARMS Upper Extremity Model/Model/4.1/MOBL_ARMS_fixed_41.osim");
    

    // Model model = Model("./MoBL_ARMS_bimanual_6_2_21.osim");
    
    // Model model = Model("../models/os4bimanual/os4bimanual.osim");
    Model model = Model("./os4bimanual_marked.osim");
    // Model model = Model("./MOBL_ARMS_fixed_41.osim");
    model.setName("bimanual!");
    model.setUseVisualizer(true);
    BodySet bs = model.get_BodySet();
    JointSet js = model.get_JointSet();
    MarkerSet ms = model.get_MarkerSet();
    std::cout << bs;
    std::cout << "\n";
    std::cout << js;
    std::cout << "\n";
    std::cout << ms;
    std::cout << "\n";

    std::cout << "\n";


    // std::cout << bs.get("clavphant_l").getConcreteClassName() << std::endl;

    // Marker m1 = OpenSim::Marker("acromioclavicular_marker_l", bs.get("clavphant_l"), SimTK::Vec3(0, 0, 0));
    // std::cout << m1.getParentFrame();
    // std::cout << "\n";
    // std::cout << m1.get_fixed();
    // std::cout << "\n";
    // std::cout << m1.getParentFrameName();
    // std::cout << "\n";
    // m1.setParentFrameName("/bimanual/clavphant_l");
    // // m1.changeFrame(bs.get("clavphant_l"));

    // // ms.adoptAndAppend(&m1);
    // // model.updateMarkerSet(ms);
    // // std::cout << ms << std::endl;
    
    // model.addMarker(&m1);
    // MarkerSet mu = model.get_MarkerSet();
    // std::cout << mu;
    // std::cout << "\n";


    // // Get the number of joints in the model
    // int num_joints = model.getNumJoints();

    // // Loop through each joint in the model
    // for (int i = 0; i < num_joints; i++) {
    //     // Get the joint object
    //     Joint& joint = model.getJointSet().get(i);

    //     // Get the parent and child bodies of the joint
    //     const PhysicalFrame& parent_body = joint.getParentFrame();
    //     const PhysicalFrame& child_body = joint.getChildFrame();

    //     // Compute the midpoint between the parent and child bodies
    //     Vec3 midpoint = (parent_body.getMassCenter() + child_body.getMassCenter()) / 2.0;

    //     // Create a new marker at the midpoint
    //     std::string marker_name = joint.getName() + "_marker";
    //     Marker marker(marker_name, *parent_body.clone(), midpoint);

    //     // Add the marker to the model's marker set
    //     model.getMarkerSet().adoptAndAppend(&marker);
    // }

    // std::cout << bs.get("thorax");

    // PinJoint* ground = new PinJoint("ground", 
    //     model.getGround(), Vec3(0), Vec3(0), 
    //     bs.get("thorax"), Vec3(0), Vec3(0)
    // );

    // model.addJoint(ground);

    // PinJoint* elbow = new PinJoint("elbow",
    //         *humerus, Vec3(0, -0.5, 0), Vec3(0),
    //         *radius, Vec3(0, 0.5, 0), Vec3(0));
    // std::cout << bs;
    // std::cout << model.get_BodySet();

    // // Create two links, each with a mass of 1 kg, center of mass at the body's
    // // origin, and moments and products of inertia corresponding to an
    // // ellipsoid with radii of 0.1, 0.5 and 0.1, in the x, y and z directions,
    // // respectively.
    // OpenSim::Body* humerus = new OpenSim::Body(
    //     "humerus", 1.0, Vec3(0), Inertia(0.052, 0.004, 0.052));
    // OpenSim::Body* radius  = new OpenSim::Body(
    //     "radius",  1.0, Vec3(0), Inertia(0.052, 0.004, 0.052));

    // // Connect the bodies with pin joints. Assume each body is 1 m long.
    // PinJoint* shoulder = new PinJoint("shoulder",
    //         // Parent body, location in parent, orientation in parent.
    //         model.getGround(), Vec3(0), Vec3(0),
    //         // Child body, location in child, orientation in child.
    //         *humerus, Vec3(0, 0.5, 0), Vec3(0));
    // PinJoint* elbow = new PinJoint("elbow",
    //         *humerus, Vec3(0, -0.5, 0), Vec3(0),
    //         *radius, Vec3(0, 0.5, 0), Vec3(0));

    // // Add a muscle that flexes the elbow.
    // Millard2012EquilibriumMuscle* biceps = new
    //     Millard2012EquilibriumMuscle("biceps", 200, 0.6, 0.55, 0);
    // biceps->addNewPathPoint("origin",    *humerus, Vec3(0, 0.3, 0));
    // biceps->addNewPathPoint("insertion", *radius,  Vec3(0, 0.2, 0));

    // // Add a controller that specifies the excitation of the muscle.
    // PrescribedController* brain = new PrescribedController();
    // brain->addActuator(*biceps);
    // // Muscle excitation is 0.3 for the first 0.5 seconds, then increases to 1.
    // brain->prescribeControlForActuator("biceps",
    //         new StepFunction(0.5, 3, 0.3, 1));

    // // Add components to the model.
    // model.addBody(humerus);    model.addBody(radius);
    // model.addJoint(shoulder);  model.addJoint(elbow);
    // model.addForce(biceps);
    // model.addController(brain);

    // // Add a console reporter to print the muscle fiber force and elbow angle.
    // ConsoleReporter* reporter = new ConsoleReporter();
    // reporter->set_report_time_interval(1.0);
    // reporter->addToReport(biceps->getOutput("fiber_force"));
    // reporter->addToReport(
    //     elbow->getCoordinate(PinJoint::Coord::RotationZ).getOutput("value"),
    //     "elbow_angle");
    // model.addComponent(reporter);

    // // Add display geometry.
    // Ellipsoid bodyGeometry(0.1, 0.5, 0.1);
    // bodyGeometry.setColor(Gray);
    // // Attach an ellipsoid to a frame located at the center of each body.
    // PhysicalOffsetFrame* humerusCenter = new PhysicalOffsetFrame(
    //     "humerusCenter", *humerus, Transform(Vec3(0)));
    // humerus->addComponent(humerusCenter);
    // humerusCenter->attachGeometry(bodyGeometry.clone());
    // PhysicalOffsetFrame* radiusCenter = new PhysicalOffsetFrame(
    //     "radiusCenter", *radius, Transform(Vec3(0)));
    // radius->addComponent(radiusCenter);
    // radiusCenter->attachGeometry(bodyGeometry.clone());

    // Configure the model.

    model.finalizeConnections();

    std::cout << "HEWHEWEW";
    std::cout << "\n";


    State& state = model.initSystem();

    
    // Fix the shoulder at its default angle and begin with the elbow flexed.
    // shoulder->getCoordinate().setLocked(state, true);
    // elbow->getCoordinate().setValue(state, 0.5 * Pi);
    model.equilibrateMuscles(state);

    // Configure the visualizer.
    model.updMatterSubsystem().setShowDefaultGeometry(true);
    Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundType(viz.SolidColor);
    viz.setBackgroundColor(White);



    // print model in osim format
    model.print("os4bimanual_marked.osim");

    // Simulate.
    simulate(model, state, 5.0);

    return 0;
};