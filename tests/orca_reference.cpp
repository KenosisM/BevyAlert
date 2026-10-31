// Numeric reference driver for unchanged RVO2 v2.0.2 sources, commit
// c96a6ec8328492bae0721e59774026a119a60b4a. Build instructions: docs/ORCA.md.
// This driver contains input/output plumbing, not a second ORCA implementation.
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "Agent.h"

int main() {
    std::cout << std::setprecision(10);
    std::string row;
    while (std::getline(std::cin,row)) {
        if (row.empty() || row[0]=='#') continue;
        std::replace(row.begin(),row.end(),',',' ');
        std::istringstream in(row);std::string kind;unsigned id;in >> kind >> id;
        if (kind=="lp") {
            unsigned hard,n;float radius,px,py;in >> hard >> radius >> px >> py >> n;
            std::vector<RVO::Line> lines;
            for (unsigned i=0;i<n;++i) {
                float nx,ny,offset;in >> nx >> ny >> offset;
                const RVO::Vector2 normal=RVO::normalize(RVO::Vector2(nx,ny));
                RVO::Line line;line.direction=RVO::Vector2(normal.y(),-normal.x());line.point=normal*offset;lines.push_back(line);
            }
            if (!in) return 2;
            RVO::Vector2 velocity;const size_t failed=RVO::linearProgram2(lines,radius,RVO::Vector2(px,py),false,velocity);
            if (failed<lines.size() && failed>=hard) RVO::linearProgram3(lines,hard,failed,radius,velocity);
            std::cout << "lp," << id << ',' << failed << ',' << velocity.x() << ',' << velocity.y() << '\n';
        } else if (kind=="pair") {
            float horizon,dt,ax,ay,avx,avy,ar,bx,by,bvx,bvy,br,cap,pax,pay,pbx,pby;
            in >> horizon >> dt >> ax >> ay >> avx >> avy >> ar >> bx >> by >> bvx >> bvy >> br >> cap >> pax >> pay >> pbx >> pby;
            if (!in) return 3;
            RVO::RVOSimulator sim;sim.setTimeStep(dt);
            sim.addAgent(RVO::Vector2(ax,ay),100.0f,1,horizon,horizon,ar,cap,RVO::Vector2(avx,avy));
            sim.addAgent(RVO::Vector2(bx,by),100.0f,1,horizon,horizon,br,cap,RVO::Vector2(bvx,bvy));
            sim.setAgentPrefVelocity(0,RVO::Vector2(pax,pay));sim.setAgentPrefVelocity(1,RVO::Vector2(pbx,pby));sim.doStep();
            const RVO::Line &line=sim.getAgentORCALine(0,0);
            const RVO::Vector2 normal(-line.direction.y(),line.direction.x());
            std::cout << "pair," << id << ',' << normal.x() << ',' << normal.y() << ',' << normal*line.point << ','
                << sim.getAgentVelocity(0).x() << ',' << sim.getAgentVelocity(0).y() << ','
                << sim.getAgentVelocity(1).x() << ',' << sim.getAgentVelocity(1).y() << '\n';
        } else return 4;
    }
}
