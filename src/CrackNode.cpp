#include "CrackNode.h"

CrackNode::CrackNode(int level, std::shared_ptr<Vertex> left_end, std::shared_ptr<Vertex> right_end)
: level(level), left_end(left_end), right_end(right_end)
{

}
