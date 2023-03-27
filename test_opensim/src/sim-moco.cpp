#include <OpenSim/OpenSim.h>
using namespace SimTK;
using namespace OpenSim;

int main() {
    Model model = Model("./os4bimanual.osim");
    model.setName("bimanual");
    model.setUseVisualizer(true);
    BodySet bs = model.get_BodySet();
    JointSet js = model.get_JointSet();
    std::cout << bs;
    std::cout << "\n";
    std::cout << js;
    std::cout << "\n";

    MocoStudy mocoStudy = new 
    // std::cout << bs.get("thorax");

    // PinJoint* ground = new PinJoint("ground", 
    //     model.getGround(), Vec3(0), Vec3(0), 
    //     bs.get("thorax"), Vec3(0), Vec3(0)
    // );

    // model.addJoint(ground);

    // Configure the model.
    State& state = model.initSystem();

    // print model in osim format
    // model.print("os4bimanual.osim");
    
    model.equilibrateMuscles(state);

    // Configure the visualizer.
    model.updMatterSubsystem().setShowDefaultGeometry(true);
    Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundType(viz.SolidColor);
    viz.setBackgroundColor(White);

    // Simulate.
    simulate(model, state, 5.0);

    return 0;
};