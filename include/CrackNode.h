//
// Created by Victor Hu on 2025/12/22.
//

#ifndef HAIR2D_CRACKNODE_H
#define HAIR2D_CRACKNODE_H

#include "Vertex.h"
#include <memory>
#include <vector>

class CrackNode {
public:
    int level;
    std::shared_ptr<Vertex> left_end;
    std::shared_ptr<Vertex> right_end;

    CrackNode(int level, std::shared_ptr<Vertex> left_end, std::shared_ptr<Vertex> right_end);
};



#endif //HAIR2D_CRACKNODE_H